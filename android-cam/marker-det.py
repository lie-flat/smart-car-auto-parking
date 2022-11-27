import cv2 as cv
import cv2.aruco as aruco
import pickle
import numpy as np
from boarddef import BOARD_DEFINITION, MARKER_SIZE, ARUCO_TEST_BOARD_DEFINITION, ARUCO_TEST_BOARD_IDS
from camera_info import CAMERA_MAT, DIST_COEFFS
from video_src import vid
from positioning import TRANSLATION, ROTATION, MAP_LEN_X, MAP_LEN_Y, MAP_FACTOR


# z(out) -> X
#  |    unit: m
# \ /
#  Y
CHARUCO = False
DETECT_BOARD = 'final'

# DETECTION_PARAM = aruco.DetectorParameters_create()

rots, trans = [], []

if CHARUCO:
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    from boarddef import CHARUCO_BOARD
    BOARD = CHARUCO_BOARD

else:
    if DETECT_BOARD == 'final':
        from boarddef import FINAL_BOARD_DICT, FINAL_BOARD
        dic = FINAL_BOARD_DICT
        BOARD = FINAL_BOARD
    else:
        dic = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        if DETECT_BOARD == 'board':
            BOARD = aruco.Board_create(
                BOARD_DEFINITION, dic, np.array([369, 518, 766, 22]))
        elif DETECT_BOARD == 'test':
            BOARD = aruco.Board_create(
                ARUCO_TEST_BOARD_DEFINITION, dic, ARUCO_TEST_BOARD_IDS)

world_map = 255 * np.ones((MAP_LEN_Y,MAP_LEN_X,3), dtype="uint8")

while True:
    ret, frame = vid.read()
    # print(frame.shape)
    if not ret:
        raise Exception("Failed to read image!")
    corners, ids, rejected_points = aruco.detectMarkers(frame, dic)
    if ids is not None and len(ids) >= 4:
        # print(ids)
        aruco.drawDetectedMarkers(frame, corners, ids)
        if DETECT_BOARD:
            if not CHARUCO:
                valid_cnt, rotation, translation = aruco.estimatePoseBoard(
                    corners, ids, BOARD, CAMERA_MAT, DIST_COEFFS, None, None)
                if valid_cnt > 0:
                    cv.drawFrameAxes(frame, CAMERA_MAT, DIST_COEFFS,
                                     rotation, translation, 0.08, 6)
                rotation_camera, _ = cv.Rodrigues(rotation)
                # print(f"CAMERA rot: {rotation_camera}")
                # print(f"CAMERA trans: {translation}")
                rotation_world = ROTATION @ rotation_camera
                translation_world = ROTATION @ translation + TRANSLATION.reshape(3, 1)
                # print(f"ROT: {rotation_world}")
                # print(f"TRANS: {translation_world}")
                pos = (int(translation_world[0] * MAP_FACTOR), int(translation_world[1] * MAP_FACTOR))
                print(pos)
                world_map = cv.circle(world_map, pos, 5, (255,0,0), 5)
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
    cv.imshow("world", world_map)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv.destroyAllWindows()

with open("rottrans.pkl", "wb") as f:
    pickle.dump((rots, trans), f)