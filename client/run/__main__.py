# 旧的运行脚本。。。修改自上次大作业

from functools import partial
from multiprocessing.shared_memory import SharedMemory
import cv2 as cv
import logging
import numpy as np


from ..cv import init_trackbars, init_servo_angle_predictor, stack_images
from ..controller import connect_to_board, control, buzz as buzz_raw
from ..camera import CameraReader, CVReader
from ..config import MOTION, IS_RASPBERRYPI, \
    SHM_IMG_WARP_NAME, SHM_IMG_RESULT_NAME, SHM_NP_DTYPE, \
    IMG_RESULT_SHAPE, IMG_WARP_SHAPE

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
INITIAL_TRACKBAR_VALUES = [61, 200, 30, 240]
DEBUG = True
SIGN_DET = False

predict_servo_angle = init_servo_angle_predictor(10, 5/0.3, (2.5, 12.5), DEBUG)


def create_shared_memory_nparray(data, name):
    d_size = np.dtype(SHM_NP_DTYPE).itemsize * np.prod(data.shape)
    shm = SharedMemory(create=True, size=d_size, name=name)
    dst = np.ndarray(shape=data.shape, dtype=SHM_NP_DTYPE, buffer=shm.buf)
    dst[:] = data[:]
    return shm


img_result_shm = create_shared_memory_nparray(
    np.zeros(IMG_RESULT_SHAPE, dtype=SHM_NP_DTYPE), SHM_IMG_RESULT_NAME)
img_warp_shm = create_shared_memory_nparray(
    np.zeros(IMG_WARP_SHAPE, dtype=SHM_NP_DTYPE), SHM_IMG_WARP_NAME)
img_result = np.ndarray(
    shape=IMG_RESULT_SHAPE, dtype=SHM_NP_DTYPE, buffer=img_result_shm.buf)
img_warp = np.ndarray(
    shape=IMG_WARP_SHAPE, dtype=SHM_NP_DTYPE, buffer=img_warp_shm.buf)


def main(camera, detector, width, height, initialTrackbarValues, ctrl, buzz):
    log = logging.getLogger()
    createTrackbars, readTrackbars = init_trackbars(
        initialTrackbarValues, width, height)
    createTrackbars()
    while True:
        log.info("Before reading")
        img = camera.read()
        log.info("After reading")
        points = readTrackbars()
        if DEBUG:
            servo, [imgWarpPoints, img_warp[:], imgHist,
                    img_result[:]] = predict_servo_angle(img, points)
        else:
            servo = predict_servo_angle(img, points)
        ctrl(servo=servo)
        if detector is not None:
            detected = detector.predict(img)
            for cat, result in detected.items():
                print(cat)
                cv.rectangle(img_result, (result['xmin'], result['ymin']),
                             (result['xmax'], result['ymax']), (0, 0, 255), 2)
                if cat == 'warning':
                    buzz(2000, 500)
                elif cat == 'prohibitory':
                    buzz(4000, 500)
                else:  # 'mandatory'
                    buzz(6000, 500)
        if DEBUG:
            stack = stack_images(
                1, [[img_warp, imgWarpPoints], [imgHist, img_result]])
            cv.imshow("stack", stack)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    # Using OpenCV to capture /dev/video0 on RaspberryPi
    # OR you can use CameraReader for ESP32-CAM
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s#%(levelname)s:%(message)s')
    devices = connect_to_board()
    if not IS_RASPBERRYPI and SIGN_DET:
        from ai.det import DetModel
        detector = DetModel()
    else:
        detector = None
    boardIP = devices['board']
    ctrl = partial(control, boardIP) if MOTION else lambda **_: None
    # motorSpeed 30~45 when using esp32cam
    ctrl(servo=7.5, motorA=36, motorB=0)
    buzz = partial(buzz_raw, boardIP)
    if IS_RASPBERRYPI:
        camera = CVReader(CAMERA_WIDTH, CAMERA_HEIGHT)
    else:
        camera = CameraReader(devices['esp32-cam'])
    try:
        main(camera, detector, CAMERA_WIDTH, CAMERA_HEIGHT,
             INITIAL_TRACKBAR_VALUES, ctrl, buzz)
    finally:
        img_result_shm.close()
        img_result_shm.unlink()
        img_warp_shm.close()
        img_warp_shm.unlink()
        # Reset servo and stop the car
        ctrl(servo=7.5, motorA=0, motorB=0)
