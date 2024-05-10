from std_msgs.msg import UInt8
from interfaces.action import MoveHand
from rclpy.node import Node
from rclpy.action import ActionServer
from enum import Enum
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import threading

class MyCobotState(Enum):
    IDLE = 0
    HANDLESTART = 1
    HANDLESTARTTOEND = 2
    HANDLEWORK = 3
    BACKING = 4

class AbstractMyCobotDriver(Node):
    def __init__(self, name, mc):
        super().__init__(name)
        self.rhand_lock = threading.Lock()
        self.mc = mc
        self.action_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveHand,
            'handleStartSpace',
            self.execute_handle_startspace,
            callback_group=self.action_cb_group
        )

        self._action_server = ActionServer(
            self,
            MoveHand,
            'handleStartToEndSpace',
            self.execute_handle_start_to_endspace,
            callback_group=self.action_cb_group
        )

        self._action_server = ActionServer(
            self,
            MoveHand,
            'handleWorkSpace',
            self.execute_handle_workspace,
            callback_group=self.action_cb_group
        )

        self._action_server = ActionServer(
            self,
            MoveHand,
            'handleReturnToInit',
            self.execute_handle_return,
            callback_group=self.action_cb_group
        )

        self.publisher_ = self.create_publisher(UInt8, '/spinningfactory/mycobotstate', 10, callback_group=self.publisher_cb_group)
        self.timer = self.create_timer(0.5, self.publish_state, callback_group=self.publisher_cb_group)
        self.current_task = MyCobotState.IDLE

        self.get_logger().info(name + " has started.")
        

    def execute_handle_startspace(self, goal_handle):
        with self.rhand_lock:
            self.get_logger().info(str(goal_handle) + "action received, aproaching startspace")
            self.current_task = MyCobotState.HANDLESTART

    def execute_handle_start_to_endspace(self, goal_handle):
        with self.rhand_lock:
            self.get_logger().info(str(goal_handle) + "action received, aproaching startspace")
            self.current_task = MyCobotState.HANDLESTARTTOEND
    
    def execute_handle_workspace(self, goal_handle):
        with self.rhand_lock:
            self.get_logger().info(str(goal_handle) + "action received, aproaching workspace")
            self.current_task = MyCobotState.HANDLEWORK

    def execute_handle_return(self, goal_handle):
        with self.rhand_lock:
            self.get_logger().info(str(goal_handle) + "action received, returning")
            self.current_task = MyCobotState.BACKING
 
    def publish_state(self):
            msg = UInt8()
            msg.data = self.current_task.value
            self.publisher_.publish(msg)