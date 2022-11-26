import cv2 as cv
import cv2.aruco as aruco
import numpy as np
from boarddef import BOARD_DEFINITION, MARKER_SIZE, ARUCO_TEST_BOARD_DEFINITION, ARUCO_TEST_BOARD_IDS
from camera_info import CAMERA_MAT, DIST_COEFFS


ip = input("IP address of your phone: ")
vid = cv.VideoCapture(f"http://{ip}:4747/video?640x480")


# z(out) -> y
#  |    unit: mm
# \ /
#  x
CHARUCO = True
DETECT_BOARD = 'test'

if CHARUCO:
    dic = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    from boarddef import CHARUCO_BOARD
    BOARD = CHARUCO_BOARD

else:
    dic = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)

    if DETECT_BOARD == 'board':
        BOARD = cv.aruco.Board_create(
            BOARD_DEFINITION, dic, np.array([369, 518, 766, 22]))
    elif DETECT_BOARD == 'test':
        BOARD = cv.aruco.Board_create(
            ARUCO_TEST_BOARD_DEFINITION, dic, ARUCO_TEST_BOARD_IDS)


while True:
    ret, frame = vid.read()
    if not ret:
        raise Exception("Failed to read image!")
    corners, ids, rejected_points = cv.aruco.detectMarkers(frame, dic)
    if ids is not None and len(ids) > 0:
        # print(ids)
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
        if DETECT_BOARD:
            if not CHARUCO:
                valid_cnt, rotation, translation = cv.aruco.estimatePoseBoard(
                    corners, ids, BOARD, CAMERA_MAT, DIST_COEFFS, None, None)
                if valid_cnt > 0:
                    cv.drawFrameAxes(frame, CAMERA_MAT, DIST_COEFFS,
                                     rotation, translation, 20, 10)
                    # print(translation)
            else:
                result, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, frame, BOARD, None, None, CAMERA_MAT, DIST_COEFFS)
                if charuco_ids is not None and len(charuco_ids) > 0:
                    aruco.drawDetectedCornersCharuco(
                        frame, charuco_corners, charuco_ids, (255, 0, 0))
                    valid, rotation, translation = aruco.estimatePoseCharucoBoard(
                        charuco_corners, charuco_ids, BOARD, CAMERA_MAT, DIST_COEFFS, None, None)
                    if valid:
                        cv.drawFrameAxes(frame, CAMERA_MAT, DIST_COEFFS,
                                         rotation, translation, 20, 5)
                        # print(translation)
    cv.imshow('image', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv.destroyAllWindows()
