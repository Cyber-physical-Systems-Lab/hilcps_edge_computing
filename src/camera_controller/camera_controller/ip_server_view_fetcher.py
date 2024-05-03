import requests
import cv2
import numpy as np
from camera_controller.abstract_view_fetcher import AbstractViewFetcher


class IPServerViewFetcher(AbstractViewFetcher):
    def __init__(self, url):
        self.url = url
        super().__init__()

    def fetch_image(self):
        img_resp = requests.get(url=self.url, timeout=2)
        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
        img = cv2.imdecode(img_arr, -1)
        return self._resize(img)
