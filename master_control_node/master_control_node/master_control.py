from enum import Enum
from copy import deepcopy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.clock import Duration

import numpy as np

from std_msgs.msg import Bool, UInt8, Int8
from can_msgs.msg import MotorMsg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# Positional tolerance for achieveing a given state
ANGULAR_TOLERANCE = 1.0
LOOSE_TOLERANCE = 7.0

# gear ratio for pan and tilt motor systems
PAN_TILT_RATIO = 30.0

# center of mass for the tilt assembly measured from the rotation axis
FIRING_COM = [0.039092, 0.107406] # x, z (all in m)
FIRING_MASS = 13.6077 # kg
GRAVITY = 9.81 # m/s^2
TILT_ARBFF_COEFF = 0.03 # strength of the arbitrary feed forward

# control mode 0 is percent output
# control mode 1 is position PID
# control mode 2 is velocity PID
# control mode 7 is motion magic PID

# specified bounds for motors
MOTOR_BOUNDS = {
    'pan_motor': [-40.0, 40.0],
    'tilt_motor': [0.0, 30.0],
}

# Basic operational states for the robot
IDLE_STATE = {
    'pan_motor': MotorMsg(control_mode=7),
    'tilt_motor': MotorMsg(control_mode=7, demand=26.0),
    'left_wheel_motor': MotorMsg(control_mode=2, demand=50.0),
    'right_wheel_motor': MotorMsg(control_mode=2, demand=50.0),
    'fire_solenoid': MotorMsg(),
}

ZERO_STATE = {
    'pan_motor': MotorMsg(control_mode=7),
    'tilt_motor': MotorMsg(control_mode=7, demand=35.0),
    'left_wheel_motor': MotorMsg(control_mode=2, demand=0.0),
    'right_wheel_motor': MotorMsg(control_mode=2, demand=0.0),
    'fire_solenoid': MotorMsg(),
}

FIRE_STATE = {
    'fire_solenoid': MotorMsg(demand=1.0),
}

END_FIRE_STATE = {
    'fire_solenoid': MotorMsg(demand=1.0),
}

# Timing parameters
FIRE_TIME = 0.25 * 1000000000 # in ns
SIGNAL_CLEAR_TIME = 2.5 * 1000000000 # in ns
SPIN_PERIOD = 0.05

# expected node list to wait for before fully initializing
EXPECTED_NODES = [
    'analyzers',
    'basic_trajectory',
    'hardware_node',
    'zed_node',
    'joint_state_pub',
    'zed_translator'
]

'''
Overall control mode for the system
'''
class ControlMode(Enum):
    STATIC = 0,
    ROUTE_FIT = 1,
    TRUE_PRED = 2

'''
Container for holding a single joint representation
'''
class SingleJoint:
    position = 0.0
    velocity = 0.0
    effort = 0.0

'''
Approximate equality function
'''
def epsilonEquals(control, val, epsil):
    return control + epsil >= val and control - epsil <= val


'''
Function to place an input within certain bounds
'''
def bound(val, minVal, maxVal):
    return max(minVal,  min(maxVal, val))

class MasterControl(Node):
    def __init__(self):
        super().__init__('master_control')

        # main variables for the system
        self.state = 1
        self.modeSelect = ControlMode.STATIC
        self.signalBegin = self.get_clock().now()
        self.fireBegin = self.get_clock().now()
        self.targetState = JointState()
        self.throwTarget = deepcopy(IDLE_STATE)

        # setup timers
        self.stateTimer = self.create_timer(SPIN_PERIOD, self.stepControlStateMachine)

        # Safety systems
        self.enableSysSub = self.create_subscription(Bool, "/safety_enable", self.enableSysCb, qos_profile_system_default)
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
        self.lastMotorSend = deepcopy(IDLE_STATE)

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
        self.get_logger().info(f"Waiting for nodes {EXPECTED_NODES}")

    def stepControlStateMachine(self):
        # state 1, wait for system nodes to come online 
        if(self.state == 1):
            cmp = True

            # verify the nodes are online
            onlineNodes = self.get_node_names()
            for node in EXPECTED_NODES:
                if(not node in onlineNodes):
                    cmp = False
                    self.get_logger().warning(f"Could not find '{node}' in {onlineNodes}")
    
            if cmp:
                self.get_logger().info("All expected nodes online, transitioning to sleep state")
                self.state = 2

        # from state 2 to 3, is the wakeup group

        # state 2, sleep mode (motors disabled)
        # the machine cannot exit this without external help
        elif(self.state == 2):
            if(self.lastEnableSys):
                self.state = 3
            

        # state 3, wakeup calibration. move motors from powerd down state to idle
        elif(self.state == 3):
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
                    self.trajectoryState = JointState()
                    self.state = 5 # enter the static throw state

                self.readyForRoute = False
            

        # from state 5 to 7 is the pre-planned trajectory path

        # state 5, ocular track player until signal
        elif(self.state == 5):
            # get new point to look at
            moveState = deepcopy(IDLE_STATE) # another case of the curse of python

            if(self.trajValid):
                panDemand = self.extractJointState('pan_motor', self.trajectoryState)
                wheelDemand = self.extractJointState('left_wheel_motor', self.trajectoryState)
                # self.get_logger().info(f"Extracted panDemand: {panDemand} wheelDemand: {wheelDemand} from valid trajectory")

                # update pan for tracking and wheels due to ramp time
                moveState = self.updateDemandState('pan_motor', panDemand.position * PAN_TILT_RATIO, moveState)
                moveState = self.updateDemandState('left_wheel_motor', wheelDemand.velocity, moveState)
                moveState = self.updateDemandState('right_wheel_motor', wheelDemand.velocity, moveState)

            # move robot to follow the point if we have excceded loose track
            if(not self.machineInState(moveState, False)):
                self.moveMachine(moveState)

            # if we are in static mode, state 5 can exit to record point, or throw based on if record point is set
            # the player must signal first in either condition
            if(self.playerSignal and self.modeSelect == ControlMode.STATIC):
                if(not self.trajValid):
                    self.get_logger().error('Signaled with invalid trajectory')

                elif(self.targetState is None):
                    # mark entry time
                    self.signalBegin = self.get_clock().now()

                    # record player trajectory at signal start
                    self.targetState = self.trajectoryState
                    
                    self.get_logger().info(f'Recording to target position {self.targetState}')

                    self.state = 6 # goto record point state
                    
                else:
                    
                    panDemand = self.extractJointState('pan_motor', self.targetState) 
                    wheelDemand = self.extractJointState('left_wheel_motor', self.targetState)

                    # deal with tilt motor now 
                    tiltDemand = self.extractJointState('tilt_motor', self.targetState)

                    self.throwTarget = deepcopy(ZERO_STATE)
                    self.throwTarget = self.updateDemandState('pan_motor', panDemand.position * PAN_TILT_RATIO, self.throwTarget)
                    self.throwTarget = self.updateDemandState('tilt_motor', tiltDemand.position * PAN_TILT_RATIO, self.throwTarget, True) # need this axis to be incremental and have arb ff
                    self.throwTarget = self.updateDemandState('left_wheel_motor', wheelDemand.velocity, self.throwTarget)
                    self.throwTarget = self.updateDemandState('right_wheel_motor', wheelDemand.velocity, self.throwTarget)

                    self.get_logger().info(f'Moving to target position {self.throwTarget}')
                    
                    # clear fire begin
                    self.fireBegin = None

                    self.state = 7 # goto throw to static point state


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


        # state 7, throw preset
        elif(self.state == 7):
            # move to pre set
            
            self.moveMachine(self.throwTarget)
            
            # when ready, throw
            if(self.machineInState(self.throwTarget)):

                iterStart = self.get_clock().now()

                # handle when we just entered 
                if(self.fireBegin is None):
                    self.fireBegin = iterStart
                    self.get_logger().info("Robot moved to target position, Firing begin") 
                    self.moveMachine(FIRE_STATE)

                # handle the end of firing sequence first
                elif(iterStart - self.fireBegin > Duration(nanoseconds=int(FIRE_TIME))):
                    self.moveMachine(END_FIRE_STATE)
                    self.playerSignal = False
                    self.get_logger().info("Firing complete") 
                    self.state = 4

        # handle whole system enable / disable if told to disable or enable times out (1/4 of a second)
        if(not self.lastEnableSys or (self.get_clock().now() - self.lastEnableTime) > Duration(nanoseconds=250000000)):
            if(self.lastEnableSys): # prevent console spam in timeout event
                self.get_logger().warning("System timed out from last system wide enable")
                self.lastEnableSys = False

            self.state = 2

        # publish the current state for tracing of the system 
        self.statePub.publish(UInt8(data=self.state))

        # grab the latest tilt position and calc FF
        tiltDemand = self.extractJointState('tilt_motor', self.lastJointState)
        angle = (tiltDemand.position - ZERO_STATE['tilt_motor'].demand) / PAN_TILT_RATIO
        self.calcTiltArbFF(angle)

        # send the desired state of the motors
        self.sendMachine(self.lastMotorSend)

    '''
    Command a move from the machine to each of the motors that need to be moved
    '''
    def moveMachine(self, demandStates: dict):
        for key in list(demandStates.keys()):
            self.lastMotorSend[key] = demandStates[key]

    def sendMachine(self, demandStates: dict):
        for key in list(demandStates.keys()):
            self.motorPubs[key].publish(demandStates[key])

    '''
    calculates the arbitrary feed forward from torques for the tilt axis
    '''
    def calcTiltArbFF(self, angleFromZero: float):

        rotMatrix = np.array([
            [math.cos(angleFromZero), -math.sin(angleFromZero)],
            [math.sin(angleFromZero), math.cos(angleFromZero)]
        ])

        # calculate the transformed point
        transformedMassPt = rotMatrix.dot(np.array(FIRING_COM))
        gravityVect = np.array([0, -GRAVITY * FIRING_MASS])

        # calculate the apllied torque
        torque = np.cross(transformedMassPt, gravityVect)

        self.lastMotorSend['tilt_motor'].arb_feedforward = float(torque * -TILT_ARBFF_COEFF)

    '''
    Check if the machine is in the desired configuration yet
    '''
    def machineInState(self, demandStates: dict, isTight: bool = True) -> bool:
        for key in list(demandStates.keys()):
            if(not self.checkSingleState(key, demandStates[key], isTight)): return False

        return True

    def checkSingleState(self, joint: str, demandState: MotorMsg, isTight: bool = True) -> bool:
        try:
            # grab index of motor to check
            i = self.lastJointState.name.index(joint)

            # check equals based on control mode
            if(demandState.control_mode == 1 or demandState.control_mode == 7):
                return epsilonEquals(demandState.demand, self.lastJointState.position[i], ANGULAR_TOLERANCE if isTight else LOOSE_TOLERANCE)

            elif(demandState.control_mode == 2):
                return epsilonEquals(demandState.demand, self.lastJointState.velocity[i], ANGULAR_TOLERANCE if isTight else LOOSE_TOLERANCE)

            elif(demandState.control_mode == 0):
                return True

            else:
                self.get_logger().warning(f"Unknown control mode {demandState.control_mode}")
                return False

        except ValueError:
            self.get_logger().warning(f"Could not find {joint} in last joint state. Avaliable names: {self.lastJointState}")
            return False
    

    def extractJointState(self, joint: str, states: JointState) -> SingleJoint:
        data = SingleJoint()
        try:
            dataIdx = states.name.index(joint)
            data.position = states.position[dataIdx]
            data.velocity = states.velocity[dataIdx]
            data.effort = states.effort[dataIdx]
        except ValueError:
            self.get_logger().error(f"could not find {joint} in joint state {states}")

        return data

    '''
    Take a joint state dict and update a particular value
    handles bounding from list as well as incermental operation
    '''
    def updateDemandState(self, joint: str, demand: float, jointStates: dict, isIncremental: bool = False) -> dict:
        # if incremental increment, otherwise set
        jointStates[joint].demand = (demand + jointStates[joint].demand) if isIncremental else demand

        # Apply bounds to the motor after incremental
        if joint in MOTOR_BOUNDS:
            jointStates[joint].demand = bound(jointStates[joint].demand, MOTOR_BOUNDS[joint][0], MOTOR_BOUNDS[joint][1])

        self.get_logger().debug(f"Moving {joint} to position {demand}")

        return jointStates

    '''
    Callback functions below here for data endpoints
    '''
    def reset(self):
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
