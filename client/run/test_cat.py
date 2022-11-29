import numpy as np
import cv2 as cv
import math
from time import time

from ..cv.cat import cat
from ..camera.phone import PHONE_CAM_HEIGHT, PHONE_CAM_WIDTH
from ..camera.camera import CAR_CAM_HEIGHT, CAR_CAM_WIDTH

frame1 = np.ones((PHONE_CAM_HEIGHT, PHONE_CAM_WIDTH, 3), dtype=np.uint8) * 0
frame2 = np.ones((CAR_CAM_HEIGHT, CAR_CAM_WIDTH, 3), dtype=np.uint8) * 145
frame3 = np.ones((CAR_CAM_HEIGHT, CAR_CAM_WIDTH), dtype=np.uint8) * 255

cv.namedWindow("frame", cv.WINDOW_NORMAL)
cv.setWindowProperty("frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)

fake_dat = np.array([1.23324324, 324.3243243253425, 324.324234])

theta = math.pi / 2
rot_world = np.array([[math.cos(theta), -math.sin(theta), 0],
                     [math.sin(theta), math.cos(theta), 0],
                     [0, 0, 1]], dtype=np.float32)

# rot_world = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
trans_world = np.array([0.2, 0.4, -0.1], dtype=np.float32)
# used to record the time when we processed last frame
prev_frame_time = 0
# used to record the time at which we processed current frame
new_frame_time = 0

while True:
    new_frame_time = time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    frame = cat(frame1, frame2, frame3, trans_world,
                rot_world, fake_dat, fake_dat, fps)
    cv.imshow("frame", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
