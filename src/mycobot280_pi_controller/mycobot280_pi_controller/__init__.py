
from mycobot280_pi_controller.mycobot280_pi_controller.mock_mycobot_driver import MockMyCobotDriver
from mycobot280_pi_controller.mycobot_controller import MyCobotController
import rclpy
from rclpy.executors import MultiThreadedExecutor

STARTSPACE_TOPIC = "/spinningfactory/startspace_state"
URL = "http://192.168.151.74:8080/shot.jpg"


def start_mock_driver(args=None):
    rclpy.init(args=args)
    controller_node = MockMyCobotDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(controller_node)
    executor.spin()
    rclpy.shutdown()


def start_controller(args=None):
    rclpy.init(args=args)
    controller_node = MyCobotController()
    rclpy.spin(controller_node)
    rclpy.shutdown()
