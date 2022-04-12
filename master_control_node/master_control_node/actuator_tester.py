from typing import List
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from can_msgs.msg import MotorMsg
from sensor_msgs.msg import JointState

MONITORED_MOTORS = [
    'left_wheel_motor',
    'right_wheel_motor',
    'tilt_motor',
    'pan_motor',
    'fire_solenoid'
]

class MotorSubFrame:
    def __init__(self, name, node: Node):
        self.motorMsg = MotorMsg()

        self.sub = node.create_subscription(MotorMsg, f"/motor/{name}/demand", self.subCb, qos_profile_system_default)

    def subCb(self, msg: MotorMsg):
        self.motorMsg = msg

    def isPosition(self):
        return self.motorMsg.control_mode == 1 or self.motorMsg.control_mode == 7

    def getDemand(self):
        return self.motorMsg.demand

    

class ActuatorTester(Node):
    def __init__(self):
        super().__init__('actuator_tester')

        # joint state publisher
        self.jointStatePub = self.create_publisher(JointState, "/motor/joint_state", qos_profile_sensor_data)
        self.lastJointState = JointState()
        self.lastJointState.name = MONITORED_MOTORS
        self.lastJointState.position = [0.0] * len(MONITORED_MOTORS)
        self.lastJointState.velocity = [0.0] * len(MONITORED_MOTORS)
        self.lastJointState.effort = [0.0] * len(MONITORED_MOTORS)

        # motor subscribers
        self.motorSubs = []
        for name in MONITORED_MOTORS:
            self.motorSubs.append(MotorSubFrame(name, self))

        # timer for unified publish of last recieved state
        self.stateTimer = self.create_timer(0.05, self.pubTimerCb)

        self.get_logger().info(f"Monitoring motors {MONITORED_MOTORS}")

    def pubTimerCb(self):
        for i in range(len(self.motorSubs)):
            motorSub = self.motorSubs[i]
            if(motorSub.isPosition()):
                self.lastJointState.position[i] = motorSub.getDemand()
                self.lastJointState.velocity[i] = 0.0
            else: 
                self.lastJointState.position[i] = 0.0
                self.lastJointState.velocity[i] = motorSub.getDemand()

        self.jointStatePub.publish(self.lastJointState)
            

def main(args=None):
    rclpy.init(args=args)

    node = ActuatorTester()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


