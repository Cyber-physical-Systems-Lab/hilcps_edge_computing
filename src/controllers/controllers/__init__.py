import rclpy
from controllers.mycobot_hil_controller import MyCobotHiLController
from rclpy.executors import MultiThreadedExecutor
from signal import signal, SIGINT
import sys

def start_mycobot_hil_controller(args=None):
    rclpy.init(args=args)
    node = MyCobotHiLController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    signal(SIGINT, signal_handler)
    executor.spin()
    rclpy.shutdown()

def signal_handler(sig, frame):
    rclpy.shutdown()
    # Perform cleanup actions here
    sys.exit(0)
