from camera_controller.abstract_view_fetcher import AbstractViewFetcher
import cv2
import random


class MockViewFetcher(AbstractViewFetcher):
    def __init__(self):
        super().__init__()

    def fetch_image(self):
        random_number =  random.randint(1, 5)
        return cv2.imread(f"/home/szogabaha/src/uu/thesis/software/hilcps_edge_computing/resource/cardboard_image{random_number}.jpg")
