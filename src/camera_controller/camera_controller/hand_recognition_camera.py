from camera_controller.abstract_view_fetcher import AbstractViewFetcher
from rclpy.node import Node
from interfaces.msg import SpaceState
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from inference_sdk import InferenceHTTPClient

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="TQuDw6siaoJ2cx6Bgc1a"
)




STARTSPACE_TOPIC = "/spinningfactory/hand/startspace"

class HandRecognitionCameraController(Node):
    def __init__(self, name, topic, cam_fetcher: AbstractViewFetcher):
        super().__init__(name)
        self.camera = cam_fetcher
        self.current_state = SpaceState.EMPTY
        self.cmd_vel_pub_ = self.create_publisher(SpaceState, topic, 10)
        self.state_changer = self.create_timer(2, self.measure_board_state)
        self.get_logger().info(name + ": Hand recognition started transmitting")

    def _find_work_area(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Check if qr code is present
        right_qr_code_found = False
        qr_codes = decode(gray)
        for code in qr_codes:
            qr_data = code.data.decode("utf-8")
            if qr_data == "Top right space":
                right_qr_code_found = True
                break

        if not right_qr_code_found:
            self.get_logger().warn(self.get_name() + ": space not found.")
            #cv2.imshow("error", image)
            return  # TODO error here

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        edges = cv2.Canny(blurred, 50, 150)

        contours, hierarchy = cv2.findContours(
            edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # Filter contours
        rects = []
        for contour in contours:
            # Approximate the contour to a polygon
            polygon = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True
            )

            # Check if the polygon has at 4 sides
            if len(polygon) == 4:
                rects.append(polygon)

        return max(rects, key=cv2.contourArea)

    
    def fetch_image(self):
        try:
            return self.camera.fetch_image()
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().warn("Mocked image sent!")
            return cv2.imread("resource/cardboard_mock.jpg")

    def send_startspace_state(self):
        msg = SpaceState()
        msg.state = self.current_state

        self.cmd_vel_pub_.publish(msg)
        self.get_logger().info(f"State message published: {str(msg)}")

    def measure_board_state(self):
        image = self.fetch_image()
        self.get_logger().info("Next image received")
        results = []
        try:
            largest_rect = self._find_work_area(image)

            result = CLIENT.infer(largest_rect, model_id="palm-detection-ddox1/3")
            self.get_logger().info(f"{result}")

        except Exception as e:
            self.get_logger().error(str(e))
            results = "error"

        if results == "error":
            self.current_state = SpaceState.ERROR
        elif not results:  # empty
            self.current_state = SpaceState.EMPTY
        else:
            self.current_state = SpaceState.ITEMPLACED

        self.send_startspace_state()
