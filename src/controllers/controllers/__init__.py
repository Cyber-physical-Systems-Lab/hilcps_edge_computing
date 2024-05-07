import rclpy
from controllers.mycobot_hil_controller import MyCobotHiLController
from rclpy.executors import MultiThreadedExecutor

def start_mycobot_hil_controller(args=None):
    rclpy.init(args=args)
    node = MyCobotHiLController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()