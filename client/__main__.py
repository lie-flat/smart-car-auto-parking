from functools import partial
import cv2
import logging


from .cv import init_trackbars, init_servo_angle_predictor, stack_images
from .controller import connect_to_board, control, buzz as buzz_raw
from .camera import CameraReader, CVReader
from .camera.camera import CAR_CAM_HEIGHT, CAR_CAM_WIDTH
from .config import IS_RASPBERRYPI

INITIAL_TRACKBAR_VALUES = [61, 200, 30, 240]
MOTION = True
DEBUG = True
SIGN_DET = False

predictServoAngle = init_servo_angle_predictor(10, 5/0.3, (2.5, 12.5), DEBUG)

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
            servo, [imgWarpPoints, imgWarp, imgHist,
                    imgResult] = predictServoAngle(img, points)
        else:
            servo = predictServoAngle(img, points)
        ctrl(servo=servo)
        if detector is not None:
            detected = detector.predict(img)
            for cat, result in detected.items():
                print(cat)
                cv2.rectangle(imgResult, (result['xmin'], result['ymin']),
                              (result['xmax'], result['ymax']), (0, 0, 255), 2)
                if cat == 'warning':
                    buzz(2000, 500)
                elif cat == 'prohibitory':
                    buzz(4000, 500)
                else:  # 'mandatory'
                    buzz(6000, 500)
        if DEBUG:
            stack = stack_images(
                1, [[imgWarp, imgWarpPoints], [imgHist, imgResult]])
            cv2.imshow("stack", stack)
        if cv2.waitKey(1) & 0xFF == ord('q'):
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
    main(camera, detector, CAR_CAM_WIDTH, CAR_CAM_HEIGHT,
         INITIAL_TRACKBAR_VALUES, ctrl, buzz)
    # Reset servo and stop the car
    ctrl(servo=7.5, motorA=0, motorB=0)
