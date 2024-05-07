
from actuators.mock_mycobot_driver import MockMyCobotDriver
from actuators.mycobot_driver import MyCobotDriver
from actuators.hil_driver import HiLDriver
import rclpy
from rclpy.executors import MultiThreadedExecutor
from pymycobot import MyCobotSocket
from signal import signal, SIGINT
import sys


def start_mock_driver(args=None):
    signal(SIGINT, signal_handler)
    rclpy.init(args=args)
    driver_node = MockMyCobotDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(driver_node)
    executor.spin()
    rclpy.shutdown()

def start_driver(args=None):
    signal(SIGINT, signal_handler)
    mc = MyCobotSocket("192.168.0.223",9001)
    rclpy.init(args=args)
    driver_node = MyCobotDriver(mc = mc)
    executor = MultiThreadedExecutor()
    executor.add_node(driver_node)
    executor.spin()
    rclpy.shutdown()

def start_hil(args=None):
    signal(SIGINT, signal_handler)
    rclpy.init(args=args)
    driver_node = HiLDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(driver_node)
    executor.spin()
    rclpy.shutdown()



def signal_handler(sig, frame):
    rclpy.shutdown()
    # Perform cleanup actions here
    sys.exit(0)
