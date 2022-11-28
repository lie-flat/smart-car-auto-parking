from pyjoystick.sdl2 import Key, Joystick, run_event_loop
from controller import connect_to_board, read_sensors, control
import logging
import time
import json
import cv2

servo = 7.5
motorA = 80
motorB = 0
imgId = 0
lastControlTime = time.time()
result = []


def print_add(joy):
    print('Joystick connected:', joy)


def print_remove(joy):
    print('Joystick removed', joy)


def key_received(key):
    global servo, motorA, motorB, lastControlTime, cap, imgId

    if key.keytype == Key.AXIS:
        if key.number == 0:
            servo = key.value * 5 + 7.5
        else:
            return
        t = time.time()
        if t - lastControlTime > 0.2:
            lastControlTime = t
            log.info(
                f"{key.number}:{key.value} Send servo={servo}")
            control(boardIP, servo, motorA, motorB)
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue
                cv2.imwrite(f'train/{imgId}.jpg', frame)
                imgId += 1
                result.append((key.value + 1)/2) # to 0~1
                break
            log.info(f"{key.number}:{key.value} OK servo={servo}")
    elif key.keytype == Key.BUTTON:
        # 停止收集训练数据
        if key.number == 1:
            with open('train/train.json', 'w') as f:
                json.dump({'label': result}, f)
            exit()

if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s [%(name)s]%(levelname)s:%(message)s')
    log = logging.getLogger("main")
    devices = connect_to_board()
    log.info(f"Found devices: {devices}")
    camIP = devices['esp32-cam']
    boardIP = devices['board']
    cap = cv2.VideoCapture(0)
    control(boardIP, servo, motorA, motorB)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
    run_event_loop(print_add, print_remove, key_received)
