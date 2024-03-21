import imutils
from cv2 import VideoCapture
from camera_controller.abstract_view_fetcher import AbstractViewFetcher


class USBViewFetcher(AbstractViewFetcher):
    def __init__(self, cam_port):
        self.cam_port = cam_port
        self.cam = VideoCapture(self.cam_port) 
  
        super().__init__()

    def fetch_image(self):
        
        # reading the input using the camera 
        result, image = self.cam.read()
        return imutils.resize(image, width=1000, height=1800)
            