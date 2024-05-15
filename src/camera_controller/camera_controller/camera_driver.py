from camera_controller.abstract_view_fetcher import AbstractViewFetcher
import mediapipe as mp
from rclpy.node import Node
from interfaces.msg import SpaceState
import cv2
import numpy as np
from std_msgs.msg import Bool
from pyzbar.pyzbar import decode
import time
import uuid
import os

HAND_RECOGNITION_TOPIC_SUFFIX = "_hand"
COLOR_RANGES = {
    #"GREEN": {
    #    "lower": np.array([40, 40, 40]),
    #    "upper": np.array([86, 255, 255]),
    #},
    "RED": {
        "lower": np.array([160, 50, 50]),
        "upper": np.array([180, 255, 255]),
    },
    "BLUE": {
        "lower": np.array([90, 100, 100]),
        "upper": np.array([130, 255, 255]),
    }
}


class CameraDriver(Node):
    def __init__(self, name, area, cam_fetcher: AbstractViewFetcher):
        super().__init__(name)
        self.camera = cam_fetcher
        self.current_state = SpaceState.EMPTY
        self.hand_over_space = False

        self.mp_hands = mp.solutions.hands.Hands()

        self.spacestate_publisher = self.create_publisher(SpaceState, area, 10) # type, topic name, queue size
        self.hand_recognition_publisher = self.create_publisher(Bool, area + HAND_RECOGNITION_TOPIC_SUFFIX, 10)
        self.state_changer = self.create_timer(0.001, self.measure_board_state)
        #self.get_logger().info(name + ": Camera started transmitting")

    # Publishers
    def send_startspace_state(self, uid):
        msg = SpaceState()
        msg.state = self.current_state
        msg.unique_id = str(uid)
        self.spacestate_publisher.publish(msg)
        self.get_logger().info(f"FINISH\tPID {os.getpid() }\tUID {uid}\tTIMESTAMP {time.time_ns()}")
        #self.get_logger().info(f"State message published: {str(msg)}")

    def send_hand_recognition_state(self, uid):
        msg = Bool()
        msg.data = self.hand_over_space

        self.hand_recognition_publisher.publish(msg)
        self.get_logger().info(f"HAND\tPID {os.getpid() }\tUID {uid}\tTIMESTAMP {time.time_ns()}")
        #self.get_logger().info(f"Hand over space published: {str(msg)}")

    # Space state recognition
    def _find_work_area(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        """
        # Check if qr code is present
        right_qr_code_found = False
        qr_codes = decode(gray)
        #cv2.imshow("error", image)
        #cv2.waitKey()
            
        for code in qr_codes:
            qr_data = code.data.decode("utf-8")
            if qr_data in ["Top right space", "Bottom left space"]:
                right_qr_code_found = True
                break

        if not right_qr_code_found:
            self.get_logger().warn(self.get_name() + ": QR not found.")
            return  # TODO error here
        """
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        edges = cv2.Canny(blurred, 50, 150)

        #cv2.imshow("Rectangles", edges)
        #cv2.waitKey()
        
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

    def _contains_color(self, image, contour, lower_boundary, upper_boundary):
        # mask image by contour
        #cv2.imshow("Rectangles", image)
        #cv2.waitKey()
        
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        
        cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
        
        # Apply the mask to the image
        masked_image = cv2.bitwise_and(image, image, mask=mask)
        #cv2.imshow("Rectangles", masked_image)
        #cv2.waitKey()
        
        
        hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
        # Mask by color
        #cv2.imshow("Rectangles", hsv)
        #cv2.waitKey()
        
        mask = cv2.inRange(hsv, lower_boundary, upper_boundary)
        
        #cv2.imshow("Rectangles", mask)
        #cv2.waitKey()
        
        matching_pixels = cv2.countNonZero(mask)
        # cv2.imshow("Rectangles", masked_image)
        return matching_pixels > 1000

    # Hand state recognition
    def _detect_hand(self, image):
        result = self.mp_hands.process(image)
        return bool(result.multi_hand_landmarks)

    def fetch_image(self):
        try:
            return self.camera.fetch_image()
        except Exception as e:
            #self.get_logger().error(str(e))
            #self.get_logger().warn("Mocked image sent!")
            return cv2.imread("resource/cardboard_mock.jpg")

    def measure_board_state(self):
        uid = uuid.uuid4()
        self.get_logger().info(f"START\tPID {os.getpid() }\tUID {uid}\tTIMESTAMP {time.time_ns()}")
        image = self.fetch_image()
        #self.get_logger().info(f"AFTERFETCH\tPID {os.getpid() }\tUID {uid}\tTIMESTAMP {time.time_ns()}")
        
        # Hand recognition
        self.hand_over_space = self._detect_hand(image)
        self.send_hand_recognition_state(uid)
        #self.get_logger().info(f"AFTERHIL\tPID {os.getpid() }\tUID {uid}\tTIMESTAMP {time.time_ns()}")
        
        #self.get_logger().info("Next image received")
        results = []
        try:
            largest_rect = self._find_work_area(image)
            for color in COLOR_RANGES:
                color_included = self._contains_color(
                    image,
                    largest_rect,
                    COLOR_RANGES[color]["lower"],
                    COLOR_RANGES[color]["upper"],
                )
                if color_included:
                    results.append(color)

            #for color in results:
                #self.get_logger().info(f"Object ({color}) detected")
            ordered_results = set(results)
        
        except Exception as e:
            #self.get_logger().error(str(e))
            ordered_results = "error"

        
        #self.get_logger().info(str(results))
        if ordered_results == "error":
            self.current_state = SpaceState.ERROR
        elif not ordered_results:  # empty
            self.current_state = SpaceState.EMPTY
        elif ordered_results == {'BLUE'}:
            self.current_state = SpaceState.BLUEPLACED
        elif ordered_results == {'RED'}:
            self.current_state = SpaceState.REDPLACED
        elif ordered_results == {'BLUE', 'RED'}: #Always in this order because it's a set
            self.current_state = SpaceState.BOTHPLACED
        
        self.send_startspace_state(uid)
        
        
