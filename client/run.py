from functools import partial
import cv2
import logging


from cv import initTrackbars, initServoAnglePredictor
from controller import connect_to_board, control, buzz as buzz_raw
from camera import CameraReader, CVReader
from ai import DetModel

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
INITIAL_TRACKBAR_VALUES = [61, 200, 30, 240]
MOTION = True

predictServoAngle = initServoAnglePredictor(10, 5/0.3, (2.5, 12.5))


def main(camera, detector, width, height, initialTrackbarValues, ctrl, buzz):
    log = logging.getLogger()
    createTrackbars, readTrackbars = initTrackbars(
        initialTrackbarValues, width, height)
    createTrackbars()
    while True:
        log.info("Before reading")
        img = camera.read()
        log.info("After reading")
        detected = detector.predict(img)
        servo = predictServoAngle(img, readTrackbars)
        ctrl(servo=servo)
        for cat, result in detected.items():
            print(cat)
            cv2.rectangle(img, (result['xmin'], result['ymin']),
                          (result['xmax'], result['ymax']), (0, 0, 255), 2)
            if cat == 'warning':
                buzz(2000, 500)
            elif cat == 'prohibitory':
                buzz(4000, 500)
            else:  # 'mandatory'
                buzz(6000, 500)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    # Using OpenCV to capture /dev/video0 on RaspberryPi
    # OR you can use CameraReader for ESP32-CAM
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s#%(levelname)s:%(message)s')
    devices = connect_to_board()
    detector = DetModel()
    boardIP = devices['board']
    ctrl = partial(control, boardIP) if MOTION else lambda **_: None
    # motorSpeed 30~45 when using esp32cam
    ctrl(servo=7.5, motorA=36, motorB=0)
    buzz = partial(buzz_raw, boardIP)
    # camera = CVReader(CAMERA_WIDTH, CAMERA_HEIGHT)
    camera = CameraReader(devices['esp32-cam'])
    main(camera, detector, CAMERA_WIDTH, CAMERA_HEIGHT,
         INITIAL_TRACKBAR_VALUES, ctrl, buzz)
    # Reset servo and stop the car
    ctrl(servo=7.5, motorA=0, motorB=0)
