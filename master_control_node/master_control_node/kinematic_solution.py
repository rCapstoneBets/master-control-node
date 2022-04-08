import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.clock import Duration

from std_msgs.msg import Bool, UInt8
from can_msgs.msg import MotorMsg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

import math

IDLE_STATE = [ # full system idle in level position
    {"dem": 00.0, "mod": 7, "name": "pan_motor"},
    {"dem": 00.5, "mod": 7, "name": "tilt_motor"},
    {"dem": 50.0, "mod": 2, "name": "left_wheel_motor"}, 
    {"dem": 50.0, "mod": 2, "name": "right_wheel_motor"},
]

FIRE_STATE = [ # trigger to fire the solenoid, and thus the ball!
    {"dem": 150.0, "mod": 0, "name": "fire_solenoid"}
]

END_FIRE_STATE = [ # trigger to fire the solenoid, and thus the ball!
    {"dem": 0.0, "mod": 0, "name": "fire_solenoid"}
]

FIRE_TIME = 1.0 * 1000000000 # in ns
FIRE_TIME_TOGGLE = 0.06 * 1000000000 # in ns

class MasterControl(Node):
    def __init__(self):
        super().__init__('kinematic_control')

        # setup timers
        self.stateTimer = self.create_timer(0.05, self.stepControlStateMachine)

        # internal vars
        self.firstIteration = True
        self.wantFire = False
        # self.isFiring = False
        self.fireBegin = self.get_clock().now()
        # self.fireStrobe = self.get_clock().now()
        self.toggleIndex = 0
        self.lastDemandState = IDLE_STATE

        self.motorPubs = {
            "left_wheel_motor": self.create_publisher(MotorMsg, "/motor/left_wheel_motor/demand", qos_profile_system_default),
            "right_wheel_motor": self.create_publisher(MotorMsg, "/motor/right_wheel_motor/demand", qos_profile_system_default),
            "tilt_motor": self.create_publisher(MotorMsg, "/motor/tilt_motor/demand", qos_profile_system_default),
            "pan_motor": self.create_publisher(MotorMsg, "/motor/pan_motor/demand", qos_profile_system_default),
            "fire_solenoid": self.create_publisher(MotorMsg, "/motor/fire_solenoid/demand", qos_profile_system_default),
        }

        self.get_logger().info("Kinematic control initalized")

    '''
    Command a move from the machine to each of the motors that need to be moved
    '''
    def moveMachine(self, demandStates):
        for i in range(len(demandStates)) :
            msg = MotorMsg()
            msg.control_mode = demandStates[i]["mod"]
            msg.demand = demandStates[i]["dem"]

            self.motorPubs[demandStates[i]["name"]].publish(msg)

    def updateDemandState(self, joint, demand):
        # find the specific joint to edit
        for state in self.lastDemandState:
            if(joint in state["name"]):
                state['dem'] = demand
                break

        self.get_logger().info(f"Moving {joint} to position {demand}")

    def stepControlStateMachine(self):
        if(self.firstIteration):
            # wake machine up
            self.moveMachine(self.lastDemandState)
            self.firstIteration = False
            return

        # handle the firing timer first
        if(self.wantFire):
            if(self.get_clock().now() > Duration(nanoseconds=int(FIRE_TIME)) + self.fireBegin):
                self.moveMachine(END_FIRE_STATE)
                self.wantFire = False
                self.toggleIndex = 0
                self.get_logger().info("Firing complete") 

            elif(self.toggleIndex % 2 == 0 and self.get_clock().now() > Duration(nanoseconds=int((self.toggleIndex + 1) * FIRE_TIME_TOGGLE)) + self.fireBegin):
                self.toggleIndex = self.toggleIndex + 1
                self.moveMachine(FIRE_STATE)

            elif(self.toggleIndex % 2 == 1 and self.get_clock().now() > Duration(nanoseconds=int(self.toggleIndex * FIRE_TIME_TOGGLE)) + self.fireBegin):
                self.toggleIndex = self.toggleIndex + 1
                self.moveMachine(END_FIRE_STATE)

        # take user input
        else:
            inputCmd = input("Enter F to fire, or K to set kinematic solution: ")
            if("f" in inputCmd.lower()):
                # fire logic
                self.get_logger().info("Firing ball")   
                self.fireBegin = self.get_clock().now()
                # self.fireStrobe = self.get_clock().now()
                self.wantFire = True   
                self.moveMachine(FIRE_STATE)
                
            elif("k" in inputCmd.lower()):
                # move logic
                print("select the joint you want to move")
                print("'' will go to the idle state")
                keys = list(self.motorPubs.keys())
                for index in range(len(keys)):
                    print(f'{index} will move {keys[index]}')

                inputCmd = input("Enter the joint to move: ")

                # handle idle as empty
                if(inputCmd == ''):
                    self.get_logger().info("Moving to idle") 
                    self.lastDemandState = IDLE_STATE

                # handle all others
                else:
                    desired = int(inputCmd)
                    demand = float(input("enter the demanded value: "))
                    for index in range(len(keys)):
                        if(desired == index):
                            joint = keys[index]
                            if('wheel' in joint):
                                self.get_logger().info("Updating both wheels") 
                                self.updateDemandState('left_wheel_motor', demand)
                                self.updateDemandState('right_wheel_motor', demand)
                            else:
                                self.updateDemandState(joint, demand)        

                self.moveMachine(self.lastDemandState)

def main(args=None):
    rclpy.init(args=args)

    node = MasterControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()