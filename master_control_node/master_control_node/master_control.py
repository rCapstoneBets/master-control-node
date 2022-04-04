from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.clock import Duration

from std_msgs.msg import Bool, UInt8
from can_msgs.msg import MotorMsg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# Positional tolerance for achieveing a given state
ANGULAR_TOLERANCE = 1.0

# control mode 0 is percent output
# control mode 1 is position PID
# control mode 2 is velocity PID

# common states for the system
IDLE_STATE = [ # full system idle in level position
    {"dem": 50.0, "mod": 1, "name": "pan_motor"},
    {"dem": 00.0, "mod": 1, "name": "tilt_motor"},
    {"dem": 50.0, "mod": 2, "name": "left_wheel_motor"}, 
    {"dem": 50.0, "mod": 2, "name": "right_wheel_motor"},
    {"dem": 0.00, "mod": 0, "name": "fire_solenoid"}
]

FIRE_STATE = [ # trigger to fire the solenoid, and thus the ball!
    {"dem": 0.5, "mod": 0, "name": "fire_solenoid"}
]


# expected node list to wait for before fully initializing
# TODO finish this list with the portenta
EXPECTED_NODES = [
    'analyzers',
    'basic_trajectory',
    'ekf_localization_node',
    'hardware_node',
    'odom_to_world_broadcaster'
]

'''
Overall control mode for the system
'''
class ControlMode(Enum):
    STATIC = 0,
    ROUTE_FIT = 1,
    TRUE_PRED = 2

'''
Approximate equality function
'''
def epsilonEquals(control, val, epsil):
    return control + epsil >= val and control - epsil <= val

class MasterControl(Node):
    def __init__(self):
        super().__init__('master_control')
        self.state = 1
        self.modeSelect = ControlMode.STATIC
        self.readyForRoute = False

        # setup timers
        self.stateTimer = self.create_timer(0.05, self.stepControlStateMachine)
        self.enableTimer = self.create_timer(0.01, self.sendSafetyEnable)
        self.enableTimer.cancel()

        # setup publishers
        self.safetyPub = self.create_publisher(Bool, "/safety_enable", qos_profile_system_default)

        self.statePub = self.create_publisher(UInt8, "/state", qos_profile_system_default)

        self.motorPubs = {
            "left_wheel_motor": self.create_publisher(MotorMsg, "/motor/left_wheel_motor/demand", qos_profile_system_default),
            "right_wheel_motor": self.create_publisher(MotorMsg, "/motor/right_wheel_motor/demand", qos_profile_system_default),
            "tilt_motor": self.create_publisher(MotorMsg, "/motor/tilt_motor/demand", qos_profile_system_default),
            "pan_motor": self.create_publisher(MotorMsg, "/motor/pan_motor/demand", qos_profile_system_default),
        }

        # setup subscribers
        self.resetSub = self.create_subscription(Bool, "/reset", self.reset, qos_profile_system_default)

        self.jointStateSub = self.create_subscription(JointState, "/motor/joint_state", self.recieveJointState, qos_profile_sensor_data)
        self.lastJointState = JointState()

        self.signalSub = self.create_subscription(Bool, "/signal", self.playerSignalCb, qos_profile_system_default)
        self.playerSignal = False

        self.enableSysSub = self.create_subscription(Bool, "/enable_sys", self.playerSignalCb, qos_profile_system_default)
        self.lastEnableSys = False
        self.lastEnableTime = Time()

        # setup services

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
            

        # state 3, wakeup calibration. move motors to limits
        elif(self.state == 3):
            if(self.enableTimer.is_canceled()):
                self.enableTimer.reset()
            
            # begin move to idle state and spin wheels
            self.moveMachine(IDLE_STATE) 

            # once we have woken up move to mode set IDLE
            if(self.machineInState(IDLE_STATE)):
                self.state = 4


        # state 4, idle for signal
        elif(self.state == 4):
            # turn solenoid off
            if(self.readyForRoute):
                if(self.modeSelect == ControlMode.ROUTE_FIT): 
                    self.state = 9 # enter the route fit state
                elif(self.modeSelect == ControlMode.TRUE_PRED):
                    self.state = 9 # enter the true prediction state
                else:
                    self.targetPoint = None # clear the last target point (if any)
                    self.state = 5 # enter the static throw state

                self.readyForRoute = False

        # from state 5 to 8 is the pre-planned trajectory path

        # state 5, ocular track player until signal
        elif(self.state == 5):

            # get new point to look at

            # move robot to follow the point
            self.moveMachine(self.moveState)

            # if we are in static mode, state 5 can exit to record point, or throw based on if record point is set
            # the player must signal first in either condition
            if(self.playerSignal and self.modeSelect == ControlMode.STATIC):
                if(self.targetPoint is None):
                    self.state = 6 # goto record point state
                else:
                    self.state = 8 # goto throw to static point state


        # state 6, record desired point
        elif(self.state == 6):
            # TODO record player pose at this time as the target point
            self.targetPoint = Pose() 

            # return to occular track state as they are walking back to the start
            # and running the route
            self.state = 5


        # state 8, throw preset
        elif(self.state == 8):
            # move to pre set
            self.moveMachine(self.moveState)
            
            # when ready, throw
            if(self.machineInState(self.moveState)):
                self.moveMachine(FIRE_STATE)# throw
                self.state = 4
                
            

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

        # handle whole system enable / disable if told to disable or enable times out (1/4 of a second)
        if(not self.lastEnableSys or (self.get_clock().now() - self.lastEnableTime) > Duration(nanoseconds=250000000)):
            self.state = 2

        # enable if we were told to and state is sleep
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
        for i in range(len(self.motorPubs)) :
            msg = MotorMsg()
            msg.control_mode = demandStates[i]["mod"]
            msg.demand = demandStates[i]["dem"]

            self.motorPubs[demandStates[i]["name"]].publish(msg)

    '''
    Check if the machine is in the desired configuration yet
    '''
    def machineInState(self, demandStates):
        cmp = True
        for state in demandStates:
            cmp = cmp and self.checkSingleState(state)

        return cmp


    def checkSingleState(self, demandState):
        try:
            # grab index of motor to check
            i = self.lastJointState.name.index(demandState["name"])
            if(demandState["mod"] == 1):
                if(len(self.lastJointState.position) > i):
                    # compare angular position in mode 1
                    return epsilonEquals(demandState["dem"], self.lastJointState.position[i], ANGULAR_TOLERANCE)
                else:
                    return False
            elif(demandState["mod"] == 2):
                if(len(self.lastJointState.velocity) > i):
                    # compare angular velocity in mode 2
                    return epsilonEquals(demandState["dem"], self.lastJointState.velocity[i], ANGULAR_TOLERANCE)
                else:
                    return False
        except ValueError:
            return False

    def reset(self):
        pass

    def recieveJointState(self, state):
        self.lastJointState = state

    def playerSignalCb(self, msg):
        self.playerSignal = msg.data

    def enableSysCb(self, msg):
        self.lastEnableSys = msg.data
        self.lastEnableTime = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    node = MasterControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
