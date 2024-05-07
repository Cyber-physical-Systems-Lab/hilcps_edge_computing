from camera_controller.camera_driver import CameraDriver
from rclpy.executors import MultiThreadedExecutor
from camera_controller.usb_view_fetcher import USBViewFetcher
from camera_controller.mock_view_fetcher import MockViewFetcher
from camera_controller.ip_server_view_fetcher import IPServerViewFetcher
import rclpy
from signal import signal, SIGINT
import sys

STARTSPACE_TOPIC = "/spinningfactory/startspace_state"
WORKSPACE_TOPIC = "/spinningfactory/workspace_state"
URL = "http://192.168.151.74:8080/shot.jpg"


def start_mock_camera(args=None):
    rclpy.init(args=args)
    camera = MockViewFetcher()
    camera_node = CameraDriver("startspace_mock", STARTSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()


def start_ip_server_camera(args=None):
    rclpy.init(args=args)
    camera = IPServerViewFetcher(URL)
    camera_node = CameraDriver("startspace_IPServer", STARTSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()

def start_usb_camera(args=None):
    rclpy.init(args=args)
    camera = USBViewFetcher(cam_port=2)
    camera_node = CameraDriver("startspace_USB", STARTSPACE_TOPIC, camera)
    executor = MultiThreadedExecutor()
    executor.add_node(camera_node)
    signal(SIGINT, signal_handler)
    executor.spin()
    rclpy.shutdown()

def work_mock_camera(args=None):
    rclpy.init(args=args)
    camera = MockViewFetcher()
    camera_node = CameraDriver("workspace_mock", WORKSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()


def work_ip_server_camera(args=None):
    rclpy.init(args=args)
    camera = IPServerViewFetcher(URL)
    camera_node = CameraDriver("workspace_IPServer", WORKSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()

def work_usb_camera(args=None):
    rclpy.init(args=args)
    camera = USBViewFetcher(cam_port=2)
    camera_node = CameraDriver("workspace_USB", WORKSPACE_TOPIC, camera)
    signal(SIGINT, signal_handler)
    rclpy.spin(camera_node)
    rclpy.shutdown()


def signal_handler(sig, frame):
    rclpy.shutdown()
    # Perform cleanup actions here
    sys.exit(0)
