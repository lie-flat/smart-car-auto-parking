# 自动收集训练数据

from datetime import datetime
from functools import partial
from os import makedirs, path
import cv2
import numpy as np
from ..cv import initTrackbars, initServoAnglePredictor
from ..controller import connect_to_board, control
import logging
from ..camera import CameraReader, CVReader

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
INITIAL_TRACKBAR_VALUES = [61, 200, 30, 240]
MOTION = True

predictServoAngle = initServoAnglePredictor(10, 5/0.3, (2.5, 12.5), True)


def main(camera, width, height, initialTrackbarValues, ctrl):
    log = logging.getLogger()
    createTrackbars, readTrackbars = initTrackbars(
        initialTrackbarValues, width, height)
    createTrackbars()
    while True:
        log.info("Before reading")
        img = camera.read()
        cv2.imwrite(path.join(dataDir, f'{datetime.now().strftime("%H-%M-%S.%f")[:-3]}.jpg'), img)
        log.info("After reading")
        servo = predictServoAngle(img, readTrackbars())
        ctrl(servo=servo)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    # Using OpenCV to capture /dev/video0 on RaspberryPi
    # OR you can use CameraReader for ESP32-CAM
    date = datetime.now()
    dataDir = f"train/{date.strftime('%Y-%m-%d-%H-%M-%S')}"
    makedirs(dataDir, exist_ok=True)
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s#%(levelname)s:%(message)s')
    devices = connect_to_board()
    boardIP = devices['board']
    ctrl = partial(control, boardIP) if MOTION else lambda **_: None
    # motorSpeed 30~45 when using esp32cam
    ctrl(servo=7.5, motorA=36, motorB=0)
    # camera = CVReader(CAMERA_WIDTH, CAMERA_HEIGHT)
    camera = CameraReader(devices['esp32-cam'])
    main(camera, CAMERA_WIDTH, CAMERA_HEIGHT,
         INITIAL_TRACKBAR_VALUES, ctrl)
    ctrl(servo=7.5, motorA=0, motorB=0)
