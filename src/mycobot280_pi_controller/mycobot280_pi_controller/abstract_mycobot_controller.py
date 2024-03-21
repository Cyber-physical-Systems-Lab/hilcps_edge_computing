from interfaces.msg import SpaceState
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class AbstractMyCobotController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.mycobot_cb_group = MutuallyExclusiveCallbackGroup()
        self.startspace_listener = self.create_subscription(
            SpaceState,
            "/spinningfactory/startspace_state",
            self.handle_startspace_callback,
            10,
            callback_group=self.mycobot_cb_group,
        )
        self.workspace_listener = self.create_subscription(
            SpaceState,
            "/spinningfactory/workspace_state",
            self.handle_workspace_callback,
            10,
            callback_group=self.mycobot_cb_group,
        )
        self.current_task = SpaceState.EMPTY
        self.get_logger().info(name + " has started.")
        

    def handle_startspace_callback(self, msg: SpaceState):
        self.get_logger().info(str(msg) + " received, aproaching startspace")
        
    def handle_workspace_callback(self, msg: SpaceState):
        self.get_logger().info(str(msg) + " received, aproaching workspace")