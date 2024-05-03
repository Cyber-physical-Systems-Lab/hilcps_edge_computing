import imutils
from cv2 import VideoCapture
import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands.Hands()

cam = VideoCapture(0) 
result, image = cam.read()
imutils.resize(image, width=1000, height=1800)

image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

results = mp_hands.process(image_rgb)

print(str(bool(results.multi_hand_landmarks)))
#if results.multi_hand_landmarks:
 #   print("Hand detected")
#else:
 #   print("No hand detected")