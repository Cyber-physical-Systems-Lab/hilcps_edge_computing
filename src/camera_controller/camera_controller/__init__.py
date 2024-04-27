from camera_controller.camera_controller import CameraController
from camera_controller.hand_recognition_camera import HandRecognitionCameraController
from camera_controller.usb_view_fetcher import USBViewFetcher
from camera_controller.mock_view_fetcher import MockViewFetcher
from camera_controller.ip_server_view_fetcher import IPServerViewFetcher
import rclpy

STARTSPACE_TOPIC = "/spinningfactory/startspace_state"
WORKSPACE_TOPIC = "/spinningfactory/workspace_state"
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

def work_mock_camera(args=None):
    rclpy.init(args=args)
    camera = MockViewFetcher()
    camera_node = CameraController("workspace_mock", WORKSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()


def work_ip_server_camera(args=None):
    rclpy.init(args=args)
    camera = IPServerViewFetcher(URL)
    camera_node = CameraController("workspace_IPServer", WORKSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()

def work_usb_camera(args=None):
    rclpy.init(args=args)
    camera = USBViewFetcher(cam_port=2)
    camera_node = CameraController("workspace_USB", WORKSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()

def start_hand_recognition_camera(args=None):
    rclpy.init(args=args)
    camera = USBViewFetcher(cam_port=0)
    camera_node = HandRecognitionCameraController("startspace_hand_USB", STARTSPACE_TOPIC, camera)
    rclpy.spin(camera_node)
    rclpy.shutdown()
    
