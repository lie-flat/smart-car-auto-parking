import cv2
import numpy as np


def getLaneMaskByColor(img):
    # Convert img to HSV
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Note: In OpenCV, Hue: [0,180), Sat: [0,255], Value: [0,255]
    lowerBlack = np.array([0, 0, 0])
    upperBlack = np.array([179, 100, 100])
    # Get mask of black
    maskBlack = cv2.inRange(imgHsv, lowerBlack, upperBlack)
    return maskBlack


def initTrackbars(initialValues, width, height):
    def pas(_): return None

    def createTrackbars():
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

    def readTrackbars():
        widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
        heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
        widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
        heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
        points = np.float32([(widthTop, heightTop), (width-widthTop, heightTop),
                            (widthBottom, heightBottom), (width-widthBottom, heightBottom)])
        return points
    return createTrackbars, readTrackbars


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


def getHistogram(img, minPer=0.1, display=False, region=1):

    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:, :], axis=0)

    maxValue = np.max(histValues)
    minValue = minPer*maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (int(x), int(img.shape[0])), (int(x), int(
                img.shape[0]-intensity//255//region)), (255, 0, 255), 1)
            cv2.circle(
                imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        return basePoint, imgHist

    return basePoint