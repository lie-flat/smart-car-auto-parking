import cv2 as cv
import numpy as np
from ..config import BOARD_DEFINITION, MARKER_SIZE, ARUCO_TEST_BOARD_DEFINITION, ARUCO_TEST_BOARD_IDS
from ..camera import get_phone_video

vid = get_phone_video()

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