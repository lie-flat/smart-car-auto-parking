import cv2 as cv
import numpy as np
from boarddef import BOARD_DEFINITION, MARKER_SIZE, ARUCO_TEST_BOARD_DEFINITION, ARUCO_TEST_BOARD_IDS

ip = input("IP address of your phone: ")
vid = cv.VideoCapture(f"http://{ip}:4747/video?640x480")
dic = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)

# z(out) -> y
#  |    unit: mm
# \ /
#  x
DETECT_BOARD = 'test'

if DETECT_BOARD == 'board':
    BOARD = cv.aruco.Board_create(
        BOARD_DEFINITION, dic, np.array([369, 518, 766, 22]))
elif DETECT_BOARD == 'test':
    BOARD = cv.aruco.Board_create(
        ARUCO_TEST_BOARD_DEFINITION, dic, ARUCO_TEST_BOARD_IDS)

CAMERA_MAT = np.array([[4, 0, 0], [0, 4, 0], [0, 0, 1]], dtype="float32")
DIST_COEFFS = np.array([1, 1, 1, 1, 1], dtype="float32")


while True:
    ret, frame = vid.read()
    if not ret:
        raise Exception("Failed to read image!")
    corners, ids, rejected_points = cv.aruco.detectMarkers(frame, dic)
    if ids is not None and len(ids) > 0:
        # print(ids)
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
        if DETECT_BOARD:
            valid_cnt, rotation, translation = cv.aruco.estimatePoseBoard(
                corners, ids, BOARD, CAMERA_MAT, DIST_COEFFS, np.zeros(3, dtype="float32"), np.zeros(3, dtype="float32"))
            if valid_cnt > 0:
                cv.drawFrameAxes(frame, CAMERA_MAT, DIST_COEFFS,
                                 rotation, translation, 20, 10)
        print(translation)
    cv.imshow('image', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv.destroyAllWindows()
