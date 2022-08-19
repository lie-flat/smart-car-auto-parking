from functools import partial
from time import sleep
import cv2
import numpy as np
from cv import getLaneMaskByColor, initTrackbars, warp, getHistogram, drawPoints, stackImages
from controller import connect_to_board, control
import logging
from camera import CameraReader, CVReader

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
INITIAL_TRACKBAR_VALUES = [61, 200, 30, 240]
MOTION = True

curveList = []
avgVal = 10


def getLaneCurve(img, readTrackbars, display=2):
    imgCopy = img.copy()
    imgResult = img.copy()
    # STEP 1
    imgThres = getLaneMaskByColor(img)
    # cv2.imshow("Thres", imgThres)

    # STEP 2
    hT, wT, c = img.shape
    points = readTrackbars()
    imgWarp = warp(imgThres, points, wT, hT)
    # imgWarpX = utils.warpImg(img, points, wT, hT)
    # cv2.imshow("Warp", imgWarpX)
    imgWarpPoints = drawPoints(imgCopy, points)

    # STEP 3
    middlePoint, imgHist = getHistogram(
        imgWarp, display=True, minPer=0.5, region=4)
    curveAveragePoint, imgHist = getHistogram(
        imgWarp, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    # SETP 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList)/len(curveList))

    # STEP 5
    if display != 0:
        imgInvWarp = warp(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = 450
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85),
                    cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv2.line(imgResult, (wT // 2, midY),
                 (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25),
                 (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
        #fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        #cv2.putText(imgResult, 'FPS ' + str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (230, 50, 50), 3);
    if display == 2:
        imgStacked = stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                             [imgHist, imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)
    elif display == 1:
        cv2.imshow('Resutlt', imgResult)
    curve = curve/100
    return curve


def main(camera, width, height, initialTrackbarValues, ctrl, queue):
    log = logging.getLogger()
    createTrackbars, readTrackbars = initTrackbars(
        initialTrackbarValues, width, height)
    createTrackbars()
    while True:
        log.info("Before reading")
        img = camera.read()
        log.info("After reading")
        curve = getLaneCurve(img, readTrackbars, display=2)
        print(curve)
        servo = 5 * curve/0.3
        # if abs(servo) < 2:
        #     servo = 0
        servo = np.clip(servo+7.5, 2.5, 12.5)
        queue.append(servo)
        ctrl(servo=queue.pop(0))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    # Using OpenCV to capture /dev/video0 on RaspberryPi
    # OR you can use CameraReader for ESP32-CAM
    logging.basicConfig(level=logging.INFO, format='%(asctime)s#%(levelname)s:%(message)s')
    devices = connect_to_board()
    boardIP = devices['board']
    ctrl = partial(control, boardIP) if MOTION else lambda **_: None
    ctrl(servo=7.5, motorA=30, motorB=0)
    # camera = CVReader(CAMERA_WIDTH, CAMERA_HEIGHT)
    camera = CameraReader(devices['esp32-cam'])
    queue = [7.5]  # 10 instruction backwards in time
    main(camera, CAMERA_WIDTH, CAMERA_HEIGHT,
         INITIAL_TRACKBAR_VALUES, ctrl, queue)
    ctrl(servo=7.5, motorA=0, motorB=0)
