from controller import connect_to_board, read_sensors, control
from camera import CameraReader
import logging
from logging import getLogger
import cv2
import random
from functools import partial

def main():
    log = getLogger("main")
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(name)s]%(levelname)s:%(message)s')
    devices = connect_to_board()
    log.info(f"Found devices: {devices}")
    log.info(f"Reading sensors")
    camIP = devices['esp32-cam']
    boardIP = devices['board']
    log.info(read_sensors(boardIP))
    reader = CameraReader(camIP)
    ctrl = partial(control, boardIP)
    while True:
        img = reader.read()
        cv2.imshow('img', img)
        servo = random.uniform(2.5,12.5)
        lucky = random.random() > 0.5
        motorA = random.uniform(*((0,70) if lucky else (30,100)))
        motorB = random.uniform(*((30,100) if lucky else (0,70)))
        log.info(f"servo={servo},motor={motorA},{motorB}")
        ctrl(servo, motorA, motorB)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    main()