
from interfaces.msg import SpaceState
import rclpy
from rclpy.node import Node
from pymycobot.mycobot import MyCobot
import time

class MyCobotController(Node):
    def __init__(self):
        super().__init__("mycobot_controller")
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        client = self.create_subscription(SpaceState, "/spinningfactory/startspace_state", self.handle_startspace_callback, 10)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 60)
        self.mc.set_gripper_state(0, 30)
        self.get_logger().info("mycobot_controller has started.")
        self.current_task = SpaceState.EMPTY

        
    def handle_startspace_callback(self, msg: SpaceState):
        self.get_logger().info(str(msg)+ "received")
        if msg.state == SpaceState.ITEMPLACED:
            self.mc.set_gripper_state(0, 30)
            self.mc.sync_send_coords([175.1, -181, 112.3, -100.34, 45.88, -57.43], 60,0)
            self.mc.sync_send_coords([160.7, -169.3, 37.5, -113.1, 43.45, -71.51],60,0)
            self.mc.set_gripper_state(1, 30)
            self.mc.sync_send_angles([0, 0, 0, 0, 0, 0], 60)
            self.mc.sync_send_coords([52.1, 259, 45.2, -87.3, 47.43, 83.76],60,0)
            self.mc.set_gripper_state(0, 30)
            self.mc.sync_send_angles([0, 0, 0, 0, 0, 0], 60)

        elif msg.state == SpaceState.ERROR and self.current_task != SpaceState.ITEMPLACED: #and if current goal is to pick up from startplace!
            #self.mc.send_angles([0, 0, 0, 0, 0, 0], 60)
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotController()
    rclpy.spin(node=node)
    rclpy.shutdown()

    
if __name__=="__main__":
    main()
