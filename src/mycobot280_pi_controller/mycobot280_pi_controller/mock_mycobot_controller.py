import time
from interfaces.msg import SpaceState
from mycobot280_pi_controller.abstract_mycobot_controller import AbstractMyCobotController

class MockMyCobotController(AbstractMyCobotController):
    def __init__(self):
        super().__init__("mock_mycobot_controller")
        
    def handle_startspace_callback(self, msg: SpaceState):
        super().handle_startspace_callback(msg)
        if msg.state == SpaceState.ITEMPLACED:
            self.get_logger().info("item placed, moving item")
            time.sleep(2)

        # TODO if item is good to go -> move right to the end

        elif (
            msg.state == SpaceState.ERROR and self.current_task != SpaceState.ITEMPLACED
        ):  # and if current goal is to pick up from startplace!
            self.get_logger().info("error on field, moving back")
            time.sleep(2)

    def handle_workspace_callback(self, msg: SpaceState):
        super().handle_workspace_callback(msg)
        if msg.state == SpaceState.ITEMPLACED:
            self.get_logger().info("item placed, moving item")
            time.sleep(2)

        # TODO if item is good to go -> move right to the end

        elif (
            msg.state == SpaceState.ERROR and self.current_task != SpaceState.ITEMPLACED
        ):  # and if current goal is to pick up from startplace!
            self.get_logger().info("error on field, moving back")
            time.sleep(2)