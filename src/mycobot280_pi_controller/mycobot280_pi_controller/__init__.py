
from mycobot280_pi_controller.mock_mycobot_controller import MockMyCobotController
from mycobot280_pi_controller.mycobot_controller import MyCobotController
import rclpy

STARTSPACE_TOPIC = "/spinningfactory/startspace_state"
URL = "http://192.168.151.74:8080/shot.jpg"


def start_mock_controller(args=None):
    rclpy.init(args=args)
    controller_node = MockMyCobotController()
    rclpy.spin(controller_node)
    rclpy.shutdown()


def start_controller(args=None):
    rclpy.init(args=args)
    controller_node = MyCobotController()
    rclpy.spin(controller_node)
    rclpy.shutdown()
