import cv2
import numpy as np
from .utils import draw_points


def init_servo_angle_predictor(avgWindowLen, coefficent, servoRange, debug=True):
    mid = (servoRange[0] + servoRange[1])/2
    curveList = []

    def predict_servo_angle(img, points):
        imgCopy = img.copy()
        imgResult = img.copy()
        # STEP 1
        imgThres = get_lane_mask_by_color(img)

        # STEP 2
        hT, wT, c = img.shape
        imgWarp = warp(imgThres, points, wT, hT)
        if debug:
            imgWarpPoints = draw_points(imgCopy, points)

        # STEP 3
        middlePoint = get_histogram(
            imgWarp, minPer=0.5, region=4, debug=False)
        curveAveragePoint, imgHist = get_histogram(
            imgWarp, minPer=0.9, debug=debug)
        curveRaw = curveAveragePoint - middlePoint

        # SETP 4
        curveList.append(curveRaw)
        if len(curveList) > avgWindowLen:
            curveList.pop(0)
        curve = int(sum(curveList)/len(curveList))

        # STEP 5
        if debug:
            imgInvWarp = warp(imgWarp, points, wT, hT, inv=True)
            imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
            imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
            imgLaneColor = np.zeros_like(img)
            imgLaneColor[:] = 0, 255, 0
            imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
            imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
            # cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85),
            #             cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        curve = curve/100
        servo = curve * coefficent
        servo = np.clip(servo+mid, servoRange[0], servoRange[1])
        return servo, [imgWarpPoints, imgWarp, imgHist, imgResult] if debug else servo
    return predict_servo_angle


def get_lane_mask_by_color(img):
    # Convert img to HSV
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Note: In OpenCV, Hue: [0,180), Sat: [0,255], Value: [0,255]
    lowerBlack = np.array([0, 0, 0])
    # Pi: [179, 100, 100]
    # ESP:[179, 255, 76]
    upperBlack = np.array([179, 255, 76])
    # Get mask of black
    maskBlack = cv2.inRange(imgHsv, lowerBlack, upperBlack)
    return maskBlack


def init_trackbars(initialValues, width, height):
    def pas(_): return None

    def create_trackbars():
        cv2.namedWindow("Trackbars")
        cv2.resizeWindow("Trackbars", 360, 240)
        cv2.createTrackbar("Width Top", "Trackbars",
                           initialValues[0], width//2, pas)
        cv2.createTrackbar("Height Top", "Trackbars",
                           initialValues[1], height, pas)
        cv2.createTrackbar("Width Bottom", "Trackbars",
                           initialValues[2], width//2, pas)
        cv2.createTrackbar("Height Bottom", "Trackbars",
                           initialValues[3], height, pas)

    def read_trackbars():
        widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
        heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
        widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
        heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
        points = np.float32([(widthTop, heightTop), (width-widthTop, heightTop),
                            (widthBottom, heightBottom), (width-widthBottom, heightBottom)])
        return points
    return create_trackbars, read_trackbars


def warp(img, points, w, h, inv=False):
    # 1 -- 2
    # |    |
    # 3 -- 4
    # TO
    # 0,0 -- w,0
    #  |      |
    # 0,h -- w,h
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp


def get_histogram(img, minPer=0.1, region=1, debug=True):

    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:, :], axis=0)

    maxValue = np.max(histValues)
    minValue = minPer*maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    if debug:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (int(x), int(img.shape[0])), (int(x), int(
                img.shape[0]-intensity//255//region)), (255, 0, 255), 1)
            cv2.circle(
                imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        return basePoint, imgHist

    return basePoint
