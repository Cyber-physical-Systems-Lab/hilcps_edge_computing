from configuration import STARTSPACE_TOPIC
from interfaces.msg import SpaceState
import rclpy
from rclpy.node import Node
from pymycobot.mycobot import MyCobot


class MyCobotController(Node):
    def __init__(self):
        super().__init__("mycobot_controller")
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        client = self.create_subscription(SpaceState, STARTSPACE_TOPIC, self.handle_startspace_callback, 10)
        self.mc.send_coords([0, 0, 0, 0, 0, 0], 30,0)
        self.mc.set_gripper_state(0, 30)
        self.get_logger().info("mycobot_controller has started.")

        
    def handle_startspace_callback(self, msg: SpaceState):
        if msg.state == SpaceState.ITEMPLACED:
            self.mc.send_coords([180.4, (-219.1), 29.5, (-164.32), (-1.86), (-128.16)], 30,0)
            while self.mc.is_moving():
                pass
            self.mc.set_gripper_state(1, 30)
            self.mc.send_coords([0, 0, 0, 0, 0, ], 30,0)

        elif msg.state == SpaceState.ERROR: #and if current goal is to pick up from startplace!
            self.mc.send_coords([0, 0, 0, 0, 0, ], 30,0)
        

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotController()
    rclpy.spin(node=node)
    rclpy.shutdown()

    
if __name__=="__main__":
    main()