import numpy as np
import cv2 as cv

from ..cv.cat import cat
from ..camera.phone import PHONE_CAM_HEIGHT, PHONE_CAM_WIDTH
from ..config import MAP_LEN_X, MAP_LEN_Y

frame1 = np.ones((PHONE_CAM_HEIGHT, PHONE_CAM_WIDTH, 3), dtype=np.uint8) * 0
frame2 = np.ones((PHONE_CAM_HEIGHT, PHONE_CAM_WIDTH, 3), dtype=np.uint8) * 145
frame3 = np.ones((PHONE_CAM_HEIGHT, PHONE_CAM_WIDTH, 3), dtype=np.uint8) * 255
frame4 = np.ones((MAP_LEN_X, MAP_LEN_Y, 3), dtype=np.uint8) * 255
frame5 = np.ones((MAP_LEN_X, MAP_LEN_Y, 3), dtype=np.uint8) * 0

cv.namedWindow("frame", cv.WINDOW_NORMAL)
cv.setWindowProperty("frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)

while True:
    frame = cat(frame1, frame2, frame3, frame4, frame5, None, None, None)
    cv.imshow("frame", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
