from interfaces.msg import SpaceState
from mycobot280_pi_controller.abstract_mycobot_controller import AbstractMyCobotController
import rclpy
from pymycobot.mycobot import MyCobot

class MyCobotController(AbstractMyCobotController):
    def __init__(self):
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 60)
        self.mc.set_gripper_state(0, 30)
        super().__init__("mycobot_controller")
        
    def handle_startspace_callback(self, msg: SpaceState):
        super().handle_startspace_callback(msg)
        if msg.state == SpaceState.ITEMPLACED:
            self.mc.set_gripper_state(0, 30)
            self.mc.sync_send_coords(
                [175.1, -181, 112.3, -100.34, 45.88, -57.43], 60, 0
            )
            self.mc.sync_send_coords(
                [160.7, -169.3, 37.5, -113.1, 43.45, -71.51], 60, 0
            )
            self.mc.set_gripper_state(1, 30)
            self.mc.sync_send_angles([0, 0, 0, 0, 0, 0], 60)
            self.mc.sync_send_coords([52.1, 259, 45.2, -87.3, 47.43, 83.76], 60, 0)
            self.mc.set_gripper_state(0, 30)
            self.mc.sync_send_angles([0, 0, 0, 0, 0, 0], 60)

        # TODO if item is good to go -> move right to the end

        elif (
            msg.state == SpaceState.ERROR and self.current_task != SpaceState.ITEMPLACED
        ):  # and if current goal is to pick up from startplace!
            self.mc.send_angles([0, 0, 0, 0, 0, 0], 60)

    def handle_workspace_callback(self, msg: SpaceState):
        super().handle_startspace_callback(msg)
        if msg.state == SpaceState.ITEMPLACED:
            self.mc.set_gripper_state(0, 30)
            self.mc.sync_send_coords(
                [175.1, -181, 112.3, -100.34, 45.88, -57.43], 60, 0
            )
            self.mc.sync_send_coords(
                [160.7, -169.3, 37.5, -113.1, 43.45, -71.51], 60, 0
            )
            self.mc.set_gripper_state(1, 30)
            self.mc.sync_send_angles([0, 0, 0, 0, 0, 0], 60)
            self.mc.sync_send_coords([52.1, 259, 45.2, -87.3, 47.43, 83.76], 60, 0)
            self.mc.set_gripper_state(0, 30)
            self.mc.sync_send_angles([0, 0, 0, 0, 0, 0], 60)

        elif (
            msg.state == SpaceState.ERROR and self.current_task != SpaceState.ITEMPLACED
        ):  # and if current goal is to pick up from startplace!
            self.mc.send_angles([0, 0, 0, 0, 0, 0], 60)
