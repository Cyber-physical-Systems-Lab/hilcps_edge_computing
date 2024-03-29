from camera_controller.camera_controller import CameraController
from camera_controller.usb_view_fetcher import USBViewFetcher
from camera_controller.mock_view_fetcher import MockViewFetcher
from camera_controller.ip_server_view_fetcher import IPServerViewFetcher
import rclpy

STARTSPACE_TOPIC = "/spinningfactory/startspace_state"
URL = "http://192.168.151.74:8080/shot.jpg"


def start_mock_camera(args=None):
    rclpy.init(args=args)
    camera = MockViewFetcher()
    camera_node = CameraController("startspace_mock", STARTSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()


def start_ip_server_camera(args=None):
    rclpy.init(args=args)
    camera = IPServerViewFetcher(URL)
    camera_node = CameraController("startspace_IPServer", STARTSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()

def start_usb_camera(args=None):
    rclpy.init(args=args)
    camera = USBViewFetcher(cam_port=2)
    camera_node = CameraController("startspace_USB", STARTSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()
