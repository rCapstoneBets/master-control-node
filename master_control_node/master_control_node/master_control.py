import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Bool

class MasterControl(Node):
    def __init__(self):
        super().__init__('master_control')
        self.state = 1

        # setup timers
        self.stateTimer = self.create_timer(0.1, self.stepControlStateMachine)
        self.enableTimer = self.create_timer(0.01, self.sendSafetyEnable)
        self.enableTimer.cancel()

        # setup publishers
        self.safetyPub = self.create_publisher(Bool, "/safety_enable", qos_profile_system_default)

        # setup subscribers

        # setup services
        



    def stepControlStateMachine(self):
        # state 1, wait for system nodes to come online 
        if(self.state == 1):
            pass

        # state 2, wait for internet connection for RTK service
        elif(self.state == 2):
            pass

        # state 3, wait for GPS fix, and connect from portenta
        elif(self.state == 3):
            pass

        # from state 4 to 6, is the wakeup group

        # state 4, sleep mode (motors disabled)
        elif(self.state == 4):
            pass

        # state 5, wakeup calibration. move motors to limits
        elif(self.state == 5):
            pass

        # state 6, idle for signal
        elif(self.state == 6):
            pass

        # from state 7 to 8 is the pre-planned trajectory path

        # state 7, move to preset and wait signal
        elif(self.state == 7):
            pass

        # state 8, throw preset
        elif(self.state == 8):
            pass

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

        # state 15, shutdown mode
        elif(self.state == 15):
            pass


        # any state greater than 4 is an enabled system
        # need to reset the timer if state is greater than 4
        if(self.state > 4 and self.enableTimer.is_canceled()):
            self.enableTimer.reset()
        
        # if the state is less than 4 and the timer is not cancelled, cancel it
        elif(self.state <= 4 and not self.enableTimer.is_canceled()):
            self.enableTimer.cancel()
            

    def modeSetCallback(self):
        pass

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
