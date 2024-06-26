from std_msgs.msg import UInt8
from interfaces.action import MoveHand
from rclpy.node import Node
from rclpy.action import ActionServer
from enum import Enum
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import Bool

STARTSPACE = "/spinningfactory/startspace_state"
WORKSPACE = "/spinningfactory/workspace_state"
HAND_SUFFIX = "_hand"

ACTION_TRESHOLD = 5

class HILSTATE(Enum):
    IDLE = 0
    HANDLESTART = 1
    HANDLESTARTTOEND = 2
    HANDLEWORK = 3
    ASSEMBLE = 4

class HiLDriver(Node):
    def __init__(self):
        self.startspace_hil_state = False
        self.workspace_hil_state = False
        
        super().__init__("hildriver")
        self._action_server = ActionServer(
            self,
            MoveHand,
            'handleStartSpaceHiL',
            self.execute_handle_startspace,
        )

        self._action_server = ActionServer(
            self,
            MoveHand,
            'handleStartToEndSpaceHiL',
            self.execute_handle_start_to_endspace,
        )

        self._action_server = ActionServer(
            self,
            MoveHand,
            'handleWorkSpaceHiL',
            self.execute_handle_workspace,
        )

        self._action_server = ActionServer(
            self,
            MoveHand,
            'handleAssembleHiL',
            self.execute_assemble,
        )

        self.startspace_hand_subscription = self.create_subscription(
            Bool,
            STARTSPACE + HAND_SUFFIX,
            self.startspace_hand_on_receive,
            10
        )

        self.workspace_hand_subscription = self.create_subscription(
            Bool,
            WORKSPACE + HAND_SUFFIX,
            self.workspace_hand_on_receive,
            10
        )

        self.publisher_ = self.create_publisher(UInt8, '/spinningfactory/hilstate', 10)
        self.timer = self.create_timer(0.5, self.publish_state)
        self.current_task = HILSTATE.IDLE

        self.get_logger().info("HiL driver has started.")
        

    def execute_handle_startspace(self, goal_handle):

        self.get_logger().info(str(goal_handle) + " action received, hil startspace started")
        self.current_task = HILSTATE.HANDLESTART
        self.get_logger().info(str(goal_handle)  + " - waiting for intention flagging")
        
        treshold = ACTION_TRESHOLD
        while treshold < 0:
            if self.startspace_hil_state:
                treshold -=1
        
        self.get_logger().info(str(goal_handle) + " - HiL in progress, waiting for finishing signal")
        treshold = ACTION_TRESHOLD
        while treshold < 0:
            if self.workspace_hil_state:
                treshold -=1
        
        self.get_logger().info(str(goal_handle) + " - HiL finished")
        self.current_task = HILSTATE.IDLE
        
        

    def execute_handle_start_to_endspace(self, goal_handle):
        self.get_logger().info(str(goal_handle) + " action received, hil start_to_end started")
        self.current_task = HILSTATE.HANDLESTARTTOEND
        self.get_logger().info(str(goal_handle)  + " - waiting for intention flagging")
        
        treshold = ACTION_TRESHOLD
        while treshold < 0:
            if self.startspace_hil_state:
                treshold -=1
        
        self.get_logger().info(str(goal_handle) + " - HiL in progress, waiting for finishing signal")
        treshold = ACTION_TRESHOLD
        while treshold < 0:
            if self.workspace_hil_state:
                treshold -=1
        
        self.get_logger().info(str(goal_handle) + " - HiL finished")
        self.current_task = HILSTATE.IDLE

    def execute_handle_workspace(self, goal_handle):
        self.get_logger().info(str(goal_handle) + " action received, hil workspace started")
        self.current_task = HILSTATE.HANDLEWORK
        self.get_logger().info(str(goal_handle)  + " - waiting for intention flagging")
        
        treshold = ACTION_TRESHOLD
        while treshold < 0:
            if self.workspace_hil_state:
                treshold -=1
        
        self.get_logger().info(str(goal_handle) + " - HiL in progress, waiting for finishing signal")
        treshold = ACTION_TRESHOLD
        while treshold < 0:
            if self.workspace_hil_state:
                treshold -=1
        
        self.get_logger().info(str(goal_handle) + " - HiL finished")
        self.current_task = HILSTATE.IDLE

    def execute_assemble(self, goal_handle):
        self.get_logger().info(str(goal_handle) + " action received, hil assemble started")
        self.current_task = HILSTATE.ASSEMBLE
        self.get_logger().info(str(goal_handle)  + " - waiting for intention flagging")
        
        treshold = ACTION_TRESHOLD
        while treshold < 0:
            if self.workspace_hil_state:
                treshold -=1
        
        self.get_logger().info(str(goal_handle) + " - HiL in progress, waiting for finishing signal")
        treshold = ACTION_TRESHOLD
        while treshold < 0:
            if self.workspace_hil_state:
                treshold -=1
        
        self.get_logger().info(str(goal_handle) + " - HiL finished")
        self.current_task = HILSTATE.IDLE

    def startspace_hand_on_receive(self, msg):
        self.startspace_hil_state = msg.data

    def workspace_hand_on_receive(self, msg):
        self.workspace_hil_state = msg.data    

    def publish_state(self):
            msg = UInt8()
            msg.data = self.current_task.value
            self.publisher_.publish(msg)