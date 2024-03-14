from camera_controller.camera_controller import StartSpaceCameraMock
from camera_controller.mock_view_fetcher import MockViewFetcher
from camera_controller.ip_server_view_fetcher import IPServerViewFetcher
import rclpy

STARTSPACE_TOPIC = "/spinningfactory/startspace_state"
URL = "http://192.168.151.74:8080/shot.jpg"


def start_mock_camera(args=None):
    rclpy.init(args=args)
    camera = MockViewFetcher()
    camera_node = StartSpaceCameraMock("startspace_mock", STARTSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()


def start_ip_server_camera(args=None):
    rclpy.init(args=args)
    camera = IPServerViewFetcher(URL)
    camera_node = StartSpaceCameraMock("startspace_IPServer", STARTSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()
