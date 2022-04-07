import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data

from std_msgs.msg import Bool, UInt8

class MasterControl(Node):
    def __init__(self):
        super().__init__('safety_pub')

        self.enableTimer = self.create_timer(0.01, self.sendSafetyEnable)

        # setup publishers
        self.safetyPub = self.create_publisher(Bool, "/safety_enable", qos_profile_system_default)

    def sendSafetyEnable(self):
        msg = Bool()
        msg.data = True
        self.safetyPub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = MasterControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()