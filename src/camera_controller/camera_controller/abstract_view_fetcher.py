from abc import abstractmethod
import imutils
import cv2

# Todo for now I'm only using the IpFetcher,
# But extend here for the lab camera + computer camera
class AbstractViewFetcher:
    def __init__(self):
        super().__init__()

    def _resize(self, image):
        #cv2.imshow("Rectangles", image)
        #cv2.waitKey()
        
        return image#imutils.resize(image, height=1200)

    @abstractmethod
    def fetch_image(self):
        pass
