import logging
from logging import getLogger
import cv2
import math
import numpy as np
from functools import partial

from controller import connect_to_board, read_sensors, control

minLineLength = 5
maxLineGap = 15
threshold = 9


def main():
    theta = 0
    log = getLogger("main")
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s [%(name)s]%(levelname)s:%(message)s')
    devices = connect_to_board()
    log.info(f"Found devices: {devices}")
    log.info(f"Reading sensors")
    camIP = devices['esp32-cam']
    boardIP = devices['board']
    log.info(read_sensors(boardIP))
    reader = cv2.VideoCapture(0)
    reader.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    reader.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
    ctrl = partial(control, boardIP)
    while True:
        result, img = reader.read()
        if not result:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 85, 85)
        lines = cv2.HoughLinesP(edged, 1, np.pi/180, 10,
                                minLineLength, maxLineGap)
        if lines is not None:
            for x in range(0, len(lines)):
                for x1, y1, x2, y2 in lines[x]:
                    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    theta = theta+math.atan2((y2-y1), (x2-x1))

        if theta > threshold:
            print("left")
        if theta < -threshold:
            print("right")
        if abs(theta) < threshold:
            print("straight")
        theta = 0
        cv2.imshow('img', img)
        # servo = random.uniform(2.5, 12.5)
        # lucky = random.random() > 0.5
        # motorA = random.uniform(*((0, 70) if lucky else (30, 100)))
        # motorB = random.uniform(*((30, 100) if lucky else (0, 70)))
        # log.info(f"servo={servo},motor={motorA},{motorB}")
        # ctrl(servo, motorA, motorB)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    main()
