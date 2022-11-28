from functools import partial
import cv2 as cv
import logging
import numpy as np

from .cv import init_trackbars, init_servo_angle_predictor, cat, estimate_pose_and_draw
from .controller import connect_to_board, control, buzz as buzz_raw
from .camera import CameraReader, CVReader, get_phone_video
from .camera.camera import CAR_CAM_HEIGHT, CAR_CAM_WIDTH
from .config import IS_RASPBERRYPI, MAP_LEN_X, MAP_LEN_Y, MOTION

INITIAL_TRACKBAR_VALUES = [61, 200, 30, 240]
DEBUG = True
SIGN_DET = False

predictServoAngle = init_servo_angle_predictor(10, 5/0.3, (2.5, 12.5), DEBUG)


def main(camera, detector, width, height, initialTrackbarValues, ctrl, buzz, traj_map, rect_visual):
    cv.namedWindow("frame", cv.WINDOW_NORMAL)
    cv.setWindowProperty("frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    vid = get_phone_video()
    log = logging.getLogger()
    createTrackbars, readTrackbars = init_trackbars(
        initialTrackbarValues, width, height)
    createTrackbars()
    while True:
        log.info("Before reading")
        img = camera.read()
        ret, frame = vid.read()
        if not ret:
            raise Exception("Failed to read from phone.")
        log.info("After reading")
        points = readTrackbars()
        if DEBUG:
            servo, [imgWarpPoints, imgWarp, imgHist,
                    imgResult] = predictServoAngle(img, points)
        else:
            servo = predictServoAngle(img, points)
        log.info("Servo Angle Predicted")
        ctrl(servo=servo)
        log.info("Control command sent")
        frame, traj_map, rect_visual, \
            rotation, translation,\
            rotation_world, translation_world = estimate_pose_and_draw(
                frame, traj_map, rect_visual)
        cv.imshow("res", imgResult)
        log.info("Pose estimated")
        if detector is not None:
            detected = detector.predict(img)
            for category, result in detected.items():
                print(category)
                cv.rectangle(imgResult, (result['xmin'], result['ymin']),
                             (result['xmax'], result['ymax']), (0, 0, 255), 2)
                if category == 'warning':
                    buzz(2000, 500)
                elif category == 'prohibitory':
                    buzz(4000, 500)
                else:  # 'mandatory'
                    buzz(6000, 500)
        all_concat = cat(frame, imgResult, imgWarp, traj_map, rect_visual,
                         translation_world, rotation_world, translation, rotation)
        log.info("Concatnated")
        # cv.imshow("frame", all_concat)
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
        camera = CVReader(CAR_CAM_WIDTH, CAR_CAM_HEIGHT)
    else:
        camera = CameraReader(devices['esp32-cam'])
    traj_map = 255 * np.ones((MAP_LEN_X, MAP_LEN_Y, 3), dtype="uint8")
    rect_visual = 255 * np.ones((MAP_LEN_X, MAP_LEN_Y, 3), dtype="uint8")
    main(camera, detector, CAR_CAM_WIDTH, CAR_CAM_HEIGHT,
         INITIAL_TRACKBAR_VALUES, ctrl, buzz, traj_map, rect_visual)
    # Reset servo and stop the car
    ctrl(servo=7.5, motorA=0, motorB=0)
