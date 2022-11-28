from pyjoystick.pygame import Key, Joystick, run_event_loop
from ..controller import connect_to_board, read_sensors, control
import logging
import time

servo = 7.5
motorA = 0
motorB = 0
lastControlTime = time.time()


def print_add(joy):
    print('Joystick connected')


def print_remove(joy):
    print('Joystick removed', joy)


def key_received(key):
    global servo, motorA, motorB, lastControlTime
    log.info(
        f"{key.number}:{key.value} Send servo={servo},motor={motorA},{motorB}")
    if key.keytype == Key.AXIS:
        if key.number == 0:
            servo = key.value * 5 + 7.5
        elif key.number == 1:
            if key.value >= 0:
                motorA = 0
                motorB = key.value * 70
            else:
                motorA = key.value * 70
                motorB = 0
        t = time.time()
        if t - lastControlTime > 0.1:
            lastControlTime = t
            log.info(
                f"{key.number}:{key.value} Send servo={servo},motor={motorA},{motorB}")
            control(boardIP, servo, motorA, motorB)
            log.info(
                f"{key.number}:{key.value} OK servo={servo},motor={motorA},{motorB}")


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s [%(name)s]%(levelname)s:%(message)s')
    log = logging.getLogger("main")
    devices = connect_to_board()
    log.info(f"Found devices: {devices}")
    camIP = devices['esp32-cam']
    boardIP = devices['board']
    run_event_loop(print_add, print_remove, key_received)
