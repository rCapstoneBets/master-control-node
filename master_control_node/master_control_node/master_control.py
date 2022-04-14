from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.clock import Duration

from std_msgs.msg import Bool, UInt8, Int8
from can_msgs.msg import MotorMsg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# Positional tolerance for achieveing a given state
ANGULAR_TOLERANCE = 1.0

# control mode 0 is percent output
# control mode 1 is position PID
# control mode 2 is velocity PID

# common states for the system
IDLE_STATE = [ # system idle config that consumes less power
    {"dem": 00.0, "mod": 1, "name": "pan_motor", "min_max": [-50.0, 50.0]},
    {"dem": 27.0, "mod": 1, "name": "tilt_motor", "min_max": [0.0, 40.0]},
    {"dem": 50.0, "mod": 2, "name": "left_wheel_motor"}, 
    {"dem": 50.0, "mod": 2, "name": "right_wheel_motor"},
    {"dem": 0.00, "mod": 0, "name": "fire_solenoid"}
]

ZERO_STATE = [ # full system idle in level position
    {"dem": 00.0, "mod": 1, "name": "pan_motor", "min_max": [-50.0, 50.0]},
    {"dem": 34.0, "mod": 1, "name": "tilt_motor", "min_max": [0.0, 40.0]},
    {"dem": 50.0, "mod": 2, "name": "left_wheel_motor"}, 
    {"dem": 50.0, "mod": 2, "name": "right_wheel_motor"},
    {"dem": 0.00, "mod": 0, "name": "fire_solenoid"}
]

FIRE_STATE = [ # trigger to fire the solenoid, and thus the ball!
    {"dem": 1.0, "mod": 0, "name": "fire_solenoid"}
]

END_FIRE_STATE = [ # trigger to fire the solenoid, and thus the ball!
    {"dem": 0.0, "mod": 0, "name": "fire_solenoid"}
]

FIRE_TIME = 0.25 * 1000000000 # in ns
FIRE_TIME_TOGGLE = 0.25 * 1000000000 # in ns
SIGNAL_CLEAR_TIME = 2.5 * 1000000000 # in ns

PAN_TILT_RATIO = 30.0
PAN_MOTOR_LIMITS = [-40.0, 40.0]
TILT_MOTOR_LIMITS = [0.0, 40.0]


# expected node list to wait for before fully initializing
# TODO finish this list with the portenta
EXPECTED_NODES = [
    'analyzers',
    'basic_trajectory',
    'hardware_node',
    # 'ekf_localization_node',
    # 'odom_to_world_broadcaster'
]

'''
Overall control mode for the system
'''
class ControlMode(Enum):
    STATIC = 0,
    ROUTE_FIT = 1,
    TRUE_PRED = 2

class SingleJoint:
    position = 0.0
    velocity = 0.0
    effort = 0.0

'''
Approximate equality function
'''
def epsilonEquals(control, val, epsil):
    return control + epsil >= val and control - epsil <= val

def bound(val, minVal, maxVal):
    return max(minVal,  min(maxVal, val))

class MasterControl(Node):
    def __init__(self):
        super().__init__('master_control')
        self.state = 1
        self.modeSelect = ControlMode.STATIC
        self.signalBegin = self.get_clock().now()
        self.fireBegin = self.get_clock().now()
        self.targetState = JointState()
        self.throwTarget = IDLE_STATE
        self.toggleIndex = 0

        # setup timers
        self.stateTimer = self.create_timer(0.05, self.stepControlStateMachine)
        self.enableTimer = self.create_timer(0.01, self.sendSafetyEnable)
        self.enableTimer.cancel()

        # Safety systems
        self.safetyPub = self.create_publisher(Bool, "/safety_enable", qos_profile_system_default)
        self.enableSysSub = self.create_subscription(Bool, "/enable_sys", self.enableSysCb, qos_profile_system_default)
        self.lastEnableSys = False
        self.lastEnableTime = Time()
        self.readyForRouteSub = self.create_subscription(Bool, "/route_start", self.setRouteReady, qos_profile_system_default)
        self.readyForRoute = False

        # State machine debugging
        self.statePub = self.create_publisher(UInt8, "/state_echo", qos_profile_system_default)
        self.stateSetSub = self.create_subscription(UInt8, '/state_set', self.setStateCb, qos_profile_system_default) # allow directly driving state for debug
        self.resetSub = self.create_subscription(Bool, "/reset", self.reset, qos_profile_system_default)


        # Setup publishers for actuators
        self.motorPubs = {
            "left_wheel_motor": self.create_publisher(MotorMsg, "/motor/left_wheel_motor/demand", qos_profile_system_default),
            "right_wheel_motor": self.create_publisher(MotorMsg, "/motor/right_wheel_motor/demand", qos_profile_system_default),
            "tilt_motor": self.create_publisher(MotorMsg, "/motor/tilt_motor/demand", qos_profile_system_default),
            "pan_motor": self.create_publisher(MotorMsg, "/motor/pan_motor/demand", qos_profile_system_default),
            "fire_solenoid": self.create_publisher(MotorMsg, "/motor/fire_solenoid/demand", qos_profile_system_default),
        }

        # Setup joint state subscribers
        self.jointStateSub = self.create_subscription(JointState, "/motor/joint_state", self.recieveJointState, qos_profile_sensor_data)
        self.lastJointState = JointState()
        self.trajectorySub = self.create_subscription(JointState, '/trajectory/desired_state', self.recieveTrajState, qos_profile_system_default)
        self.trajectoryState = JointState()
        self.trajectoryValidSub = self.create_subscription(Int8, '/trajectory/valid', self.validTrajCb, qos_profile_system_default)
        self.trajValid = False

        # setup signal subscriber
        self.signalSub = self.create_subscription(Bool, "/player/signal", self.playerSignalCb, qos_profile_system_default)
        self.playerSignal = False

        # Finish init stage
        self.get_logger().info("Master control initalized")
        self.get_logger().info("Waiting for nodes {}".format(EXPECTED_NODES))

    def stepControlStateMachine(self):
        # state 1, wait for system nodes to come online 
        if(self.state == 1):

            cmp = True
            onlineNodes = self.get_node_names()

            # print(onlineNodes)

            for node in EXPECTED_NODES:
                cmp = cmp and node in onlineNodes
    
            if cmp:
                self.get_logger().info("All expected nodes online, transitioning to sleep state")
                self.state = 2

        # from state 2 to 4, is the wakeup group

        # state 2, sleep mode (motors disabled)
        # the machine cannot exit this without external help
        elif(self.state == 2):
            if(not self.enableTimer.is_canceled()):
                self.enableTimer.cancel()
            

        # state 3, wakeup calibration. move motors from powerd down state to idle
        elif(self.state == 3):
            if(self.enableTimer.is_canceled()):
                self.enableTimer.reset()
                self.get_logger().info("Moving robot to idle state!")
            
            # begin move to idle state and spin wheels
            self.moveMachine(IDLE_STATE) 

            # once we have woken up move to mode set IDLE
            if(self.machineInState(IDLE_STATE)):
                self.get_logger().info("Robot achieved idle state!")
                self.state = 4


        # state 4, idle for signal
        elif(self.state == 4):
            # clear ready signal forcibly
            self.playerSignal = False

            # keep robot in idle state and spin wheels
            self.moveMachine(IDLE_STATE) 

            # turn solenoid off
            if(self.readyForRoute):
                if(self.modeSelect == ControlMode.ROUTE_FIT): 
                    self.state = 9 # enter the route fit state
                elif(self.modeSelect == ControlMode.TRUE_PRED):
                    self.state = 9 # enter the true prediction state
                else:
                    self.get_logger().info("Robot ready for static throw")
                    self.targetState = None # clear the last target point (if any)
                    self.state = 5 # enter the static throw state

                self.readyForRoute = False
            

        # from state 5 to 8 is the pre-planned trajectory path

        # state 5, ocular track player until signal
        elif(self.state == 5):

            # get new point to look at
            moveState = IDLE_STATE

            if(self.trajValid):
                panDemand = self.extractJointState('pan_motor', self.trajectoryState)
                moveState = self.updateDemandState('pan_motor', panDemand.position, moveState)

            # move robot to follow the point
            self.moveMachine(moveState)

            # if we are in static mode, state 5 can exit to record point, or throw based on if record point is set
            # the player must signal first in either condition
            if(self.playerSignal and self.modeSelect == ControlMode.STATIC):
                if(not self.trajValid):
                    self.get_logger().error(f'Signaled with invalid trajectory')

                elif(self.targetState is None):
                    # mark entry time
                    self.signalBegin = self.get_clock().now()

                    # record player trajectory at signal start
                    self.targetState = self.trajectoryState
                    
                    self.get_logger().info(f'Recording to target position {self.targetState}')
                    self.get_logger().debug(f'Waiting for signal to clear or timeout')

                    self.state = 6 # goto record point state
                    
                else:
                    self.throwTarget = self.updateDemandStates(self.targetState, ZERO_STATE)

                    self.get_logger().info(f'Moving to target position {self.throwTarget}')
                    
                    # clear fire begin
                    self.fireBegin = None

                    self.state = 8 # goto throw to static point state


        # state 6, record desired point
        elif(self.state == 6):
            # wait a few seconds for signal to clear or signal clear
            iterStart = self.get_clock().now()
            if(not self.playerSignal or iterStart - self.signalBegin > Duration(nanoseconds=int(SIGNAL_CLEAR_TIME))):
                self.get_logger().info(f'Player signal cleared')

                # Clear player signal
                self.playerSignal = False

                # return to occular track state as they are walking back to the start
                # and running the route
                self.state = 5


        # state 8, throw preset
        elif(self.state == 8):
            # move to pre set
            
            self.moveMachine(self.throwTarget)
            
            # when ready, throw
            if(self.machineInState(self.throwTarget)):

                iterStart = self.get_clock().now()
                # handle when we just entered 
                if(self.fireBegin is None):
                    self.fireBegin = iterStart
                    self.toggleIndex = 0
                    self.get_logger().info("Robot moved to target position, Firing begin") 
                    self.moveMachine(FIRE_STATE)

                # handle the end of firing sequence first
                elif(iterStart - self.fireBegin > Duration(nanoseconds=int(FIRE_TIME))):
                    self.moveMachine(END_FIRE_STATE)
                    self.toggleIndex = 0
                    self.playerSignal = False
                    self.get_logger().info("Firing complete") 
                    self.state = 4

                # handle the solenoid on case
                elif(self.toggleIndex % 2 == 0 and iterStart - self.fireBegin > Duration(nanoseconds=int((self.toggleIndex + 1) * FIRE_TIME_TOGGLE))):
                    self.toggleIndex += 1
                    self.moveMachine(FIRE_STATE)

                # handle the solenoid off case
                elif(self.toggleIndex % 2 == 1 and iterStart - self.fireBegin > Duration(nanoseconds=int(self.toggleIndex * FIRE_TIME_TOGGLE))):
                    self.toggleIndex += 1
                    self.moveMachine(END_FIRE_STATE)
                
            

        # from state 9 to 10, are the optimal trajectory mode

        # state 9, track predicted location
        elif(self.state == 9):
            pass

        # state 10, throw to predicted location
        elif(self.state == 10):
            pass

        # State 11 to 15 handle the self test routine

        # state 11, azmuith diagnostic
        if(self.state == 11):
            pass

        # state 12, elevation diagnostic
        elif(self.state == 12):
            pass

        # state 13, soft throw test
        elif(self.state == 13):
            pass

        # state 14, hard throw test
        elif(self.state == 14):
            pass

        # any state greater than 2 is an enabled system
        # need to reset the timer if state is greater than 2
        if(self.state > 2 and self.enableTimer.is_canceled()):
            self.enableTimer.reset()
        
        # if the state is less than 2 and the timer is not cancelled, cancel it
        elif(self.state <= 2 and not self.enableTimer.is_canceled()):
            self.enableTimer.cancel()

        # # handle whole system enable / disable if told to disable or enable times out (1/4 of a second)
        if(not self.lastEnableSys or (self.get_clock().now() - self.lastEnableTime) > Duration(nanoseconds=250000000)):
            if(self.lastEnableSys): # prevent console spam in timeout event
                self.get_logger().warning("System timed out from last system wide enable")
                self.lastEnableSys = False

            self.state = 2
            
        # # enable if we were told to and state is sleep
        elif(self.lastEnableSys and self.state == 2):
            self.state = 3

        # publish the current state for tracing of the system 
        stateMsg = UInt8()
        stateMsg.data = self.state
        self.statePub.publish(stateMsg)
            

    def modeSetCallback(self):
        pass

    def sendSafetyEnable(self):
        msg = Bool()
        msg.data = True
        self.safetyPub.publish(msg)

    '''
    Command a move from the machine to each of the motors that need to be moved
    '''
    def moveMachine(self, demandStates):
        for i in range(len(demandStates)) :
            msg = MotorMsg()
            msg.control_mode = demandStates[i]["mod"]
            msg.demand = demandStates[i]["dem"]

            self.motorPubs[demandStates[i]["name"]].publish(msg)

    '''
    Check if the machine is in the desired configuration yet
    '''
    def machineInState(self, demandStates) -> bool:
        cmp = True
        for state in demandStates:
            cmp = cmp and self.checkSingleState(state)

        return cmp

    def checkSingleState(self, demandState) -> bool:
        try:
            # grab index of motor to check
            i = self.lastJointState.name.index(demandState["name"])


            if(demandState["mod"] == 1 or demandState["mod"] == 7):
                return epsilonEquals(demandState["dem"], self.lastJointState.position[i], ANGULAR_TOLERANCE)

            elif(demandState["mod"] == 2):
                return epsilonEquals(demandState["dem"], self.lastJointState.velocity[i], ANGULAR_TOLERANCE)

            elif(demandState["mod"] == 0):
                return True

            else:
                self.get_logger().warning(f"Unknown control mode {demandState['mod']}")
                return False

        except ValueError:
            self.get_logger().warning(f"Could not find {demandState} in last joint state. Avaliable names: {self.lastJointState}")
            return False

    def extractJointState(self, name : str, states : JointState) -> SingleJoint:
        data = SingleJoint()

        try:
            dataIdx = states.name.index(name)
            data.position = states.position[dataIdx]
            data.velocity = states.velocity[dataIdx]
            data.effort = states.effort[dataIdx]
            return data
        except ValueError:
            self.get_logger().error(f"could not find {name} in joint state {states}")

        return data


    def updateDemandStates(self, jointState : JointState, demandStates):
        # iterate all joint states
        for state in demandStates:
            try:
                # find a match
                idx = jointState.name.index(state["name"])

                # update position
                if(state["mod"] == 1  or state["mod"] == 7):
                    if "min_max" in state:
                        # account for motor gear ratio and base offset
                        state['dem'] += bound(jointState.position[idx] * PAN_TILT_RATIO, state["min_max"][0], state["min_max"][1])
                    else:
                        # account for motor gear ratio and base offset
                        state['dem'] += jointState.position[idx] * PAN_TILT_RATIO

                # update velocity for wheel motors
                elif(state["mod"] == 2):
                    state["dem"]  = jointState.velocity[idx]

            except ValueError:
                # no match found, ignore
                pass

        return demandStates

    def updateDemandState(self, joint: str, demand: float, jointStates):
        # find the specific joint to edit
        for state in jointStates:
            if(joint in state["name"]):
                if(state["mod"] == 1  or state["mod"] == 7):
                    if "min_max" in state:
                        state['dem'] = bound(demand * PAN_TILT_RATIO, state["min_max"][0], state["min_max"][1])
                    else:
                        state['dem'] = demand * PAN_TILT_RATIO
                    break
                elif(state["mod"] == 2):
                    if "min_max" in state:
                        state['dem'] = bound(demand, state["min_max"][0], state["min_max"][1])
                    else:
                        state['dem'] = demand
                    break

        self.get_logger().debug(f"Moving {joint} to position {demand}")

        return jointStates

    def reset(self):
        self.enableTimer.cancel()
        self.playerSignal = False
        self.targetState = None
        self.state = 0

    def recieveJointState(self, state: JointState):
        self.lastJointState = state

    def recieveTrajState(self, msg: JointState):
        self.trajectoryState = msg

    def playerSignalCb(self, msg: Bool):
        self.playerSignal = msg.data

    def enableSysCb(self, msg: Bool):
        self.lastEnableSys = msg.data
        self.lastEnableTime = self.get_clock().now()

    def setStateCb(self, msg : UInt8):
        self.state = msg.data

    def setRouteReady(self, msg: Bool):
        self.readyForRoute = msg.data

    def validTrajCb(self, msg: Int8):
        self.trajValid = msg.data == 0


def main(args=None):
    rclpy.init(args=args)

    node = MasterControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
