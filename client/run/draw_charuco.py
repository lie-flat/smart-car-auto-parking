import cv2 as cv

DIC = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
board = cv.aruco.CharucoBoard_create(6, 6, 0.025, 0.02, DIC)
img = board.draw((1920,1080))
cv.imwrite("charuco.png", img)