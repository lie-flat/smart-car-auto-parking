from video_src import vid
import numpy as np
import cv2 as cv
import glob

dic = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)



# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
i = 0
while True:
    ret, img = vid.read()
    if not ret:
        raise Exception("Failed to read image!")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (10, 7), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        print("FOUND")
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.imwrite(f"b/{i}.det.png", img)
        cv.drawChessboardCorners(img, (10, 7), corners2, ret)
        cv.imshow("I", img)
        i += 1
        cv.waitKey(2)
cv.destroyAllWindows()
