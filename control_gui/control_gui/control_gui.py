from enum import Enum
from threading import Thread

import tkinter as tk
from tkinter import BOTTOM, DISABLED, LEFT, RIGHT, ttk

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Bool, UInt8


CUSTOM_WINDOW_WIDTH = 300
CUSTOM_WINDOW_HEIGHT = 200
WALL_TIMER_RATE = 0.01
LED_CYCLES = 25

class LedState(Enum):
    LED_RED=0,
    LED_BLUE=1,
    LED_YELLOW=2,
    LED_YELLOW_FLASH=3,
    LED_GREEN=4,
    LED_GREEN_FLASH=5


class NodeClass(Node):
    def __init__(self):
        super().__init__('control_gui')

        self.sendEnable = False
        self.sendReady = 0
        self.state = 0
        self.signal = False
        self.validTraj = False
        self.dispState = LedState.LED_RED

        self.ledToggleCount = LED_CYCLES
        self.ledToggleOn = True

        self.colorObj = None

        self.cbg = ReentrantCallbackGroup()

        self.timer = self.create_timer(WALL_TIMER_RATE, self.timerCallback)

        self.enablePub = self.create_publisher(Bool, 'safety_enable', qos_profile_system_default)
        self.startPub = self.create_publisher(Bool, 'route_start', qos_profile_system_default)

        self.signalSub = self.create_subscription(Bool, 'player/signal', self.signalCallback, qos_profile_system_default)
        self.validTrajSub = self.create_subscription(Bool, 'trajectory/valid', self.validTrajCallback, qos_profile_system_default)
        self.stateSub = self.create_subscription(UInt8, "state_echo", self.stateCallback, qos_profile_system_default)

        self.get_logger().info('Control GUI started')


    def timerCallback(self):
        # disabled state
        if(self.state < 5):
            self.dispState = LedState.LED_RED

        # tracking motion state
        elif(self.state == 5):
            # tracking valid traj
            if(self.validTraj):
                self.dispState = LedState.LED_BLUE

            # signaled invalid traj
            elif(self.signal and not self.dispState == LedState.LED_YELLOW_FLASH):
                self.ledToggleCount = LED_CYCLES
                self.ledToggleOn = True
                self.dispState = LedState.LED_YELLOW_FLASH

            # invalid traj
            elif(not self.signal):
                self.dispState = LedState.LED_YELLOW
        
        # recorded point
        elif(self.state == 6):
            self.dispState = LedState.LED_GREEN

        # throwing to player
        elif(self.state == 7 and not self.dispState == LedState.LED_GREEN_FLASH):
            self.ledToggleCount = LED_CYCLES
            self.ledToggleOn = True
            self.dispState = LedState.LED_GREEN_FLASH

        
        ### LED State handilng
        if(self.dispState == LedState.LED_RED):
            self.colorObj.configure(bg='red')

        elif(self.dispState == LedState.LED_BLUE):
            self.colorObj.configure(bg='blue')

        elif(self.dispState == LedState.LED_YELLOW):
            self.colorObj.configure(bg='yellow')

        elif(self.dispState == LedState.LED_YELLOW_FLASH):
            self.ledToggleCount -= 1
            if(self.ledToggleCount > 0 and self.ledToggleOn):
                self.colorObj.configure(bg='yellow')
            elif(self.ledToggleCount > 0):
                self.colorObj.configure(bg='black')
            elif(self.ledToggleCount == 0):
                self.ledToggleOn = not self.ledToggleOn
                self.ledToggleCount = LED_CYCLES
            
        elif(self.dispState == LedState.LED_GREEN):
            self.colorObj.configure(bg='green')

        elif(self.dispState == LedState.LED_GREEN_FLASH):
            self.ledToggleCount -= 1
            if(self.ledToggleCount > 0 and self.ledToggleOn):
                self.colorObj.configure(bg='green')
            elif(self.ledToggleCount > 0):
                self.colorObj.configure(bg='black')
            elif(self.ledToggleCount == 0):
                self.ledToggleOn = not self.ledToggleOn
                self.ledToggleCount = LED_CYCLES

        ### Enable publisher handling
        enableMsg = Bool(data=self.sendEnable)
        self.enablePub.publish(enableMsg)

        routeMsg = Bool(data=self.sendReady > 0)
        self.startPub.publish(routeMsg)
        if(self.sendReady > 0):
            self.sendReady -= 1


    def signalCallback(self, msg: Bool):
        self.signal = msg.data

    def validTrajCallback(self, msg: Bool):
        self.validTraj = msg.data

    def stateCallback(self, msg: UInt8):
        self.state = msg.data

    def getDisplayState(self):
        return self.dispState

    ### GUI Functions
    def toggleSendEnable(self):
        self.sendEnable = not self.sendEnable

    def getSendEnable(self) -> bool:
        return self.sendEnable

    def sendReadyCmd(self):
        self.sendReady = 5

    def setColorObj(self, obj):
        self.colorObj = obj



def buildGUI(node : NodeClass) -> tk.Tk:
    root = tk.Tk()

    root.title('Throwbot GUI')

    # compute center of sceen for window to start
    center = [
        int(root.winfo_screenwidth() / 2 - CUSTOM_WINDOW_WIDTH / 2),
        int(root.winfo_screenheight() / 2 - CUSTOM_WINDOW_HEIGHT / 2)
    ]

    root.geometry(f'{CUSTOM_WINDOW_WIDTH}x{CUSTOM_WINDOW_HEIGHT}+{center[0]}+{center[1]}')

    def changeEnable():
        node.toggleSendEnable()
        enableButton.configure(bg='green' if node.getSendEnable() else 'red')

    enableButton = tk.Button(root, text='Enable', bg='red', command=changeEnable)
    enableButton.pack()

    def readyCommand():
        node.sendReadyCmd()

    readyButton = tk.Button(root, text='Throw Ready', command=readyCommand)
    readyButton.pack()

    
    colorObj = tk.Button(root, bg='black', height=100, width=200, state=DISABLED)
    node.setColorObj(colorObj)
    colorObj.pack()

    return root


def main(args=None):
    rclpy.init(args=args)

    # spawn the node
    node = NodeClass()
    exec = MultiThreadedExecutor()
    exec.add_node(node)

    # build TK frame
    root = buildGUI(node)

    # destroy the GUI in a safe way so that ROS can stop
    def shutdown():
        exec.shutdown()
        root.destroy()

    # bind the shutdown to the window exit system
    root.protocol("WM_DELETE_WINDOW", shutdown)

    # create the thread to handle ROS
    rosThread = Thread(target=exec.spin, daemon=True)
    rosThread.start()

    # loop the GUI
    root.mainloop()

    print('Waiting for rosthread to shut down')

    # stop ros thread
    rosThread.join()

    print('Ros thread stopped')

    node.destroy_node()
    rclpy.shutdown() 

    


if __name__ == '__main__':
    main()
