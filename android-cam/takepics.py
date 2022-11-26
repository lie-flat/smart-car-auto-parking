import cv2 as cv
import numpy as np
from boarddef import BOARD_DEFINITION, MARKER_SIZE, ARUCO_TEST_BOARD_DEFINITION, ARUCO_TEST_BOARD_IDS

ip = input("IP address of your phone: ")
vid = cv.VideoCapture(f"http://{ip}:4747/video?640x480")

i = 0

while True:
    ret, frame = vid.read()
    if not ret:
        raise Exception("Failed to read image!")
    cv.imshow("FRAME", frame)
    cv.imwrite(f"pics/{i}.png", frame)
    i+=1
    if cv.waitKey(200) & 0xFF == ord('q'):
        break