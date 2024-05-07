import rclpy
from controllers.mycobot_hil_controller import MyCobotHiLController
from rclpy.executors import MultiThreadedExecutor
from signal import signal, SIGINT
import sys

def start_mycobot_hil_controller(args=None):
    rclpy.init(args=args)
    node = MyCobotHiLController()
    signal(SIGINT, signal_handler)
    rclpy.spin(node)
    rclpy.shutdown()

def signal_handler(sig, frame):
    rclpy.shutdown()
    # Perform cleanup actions here
    sys.exit(0)
