import cv2.aruco as aruco
import cv2 as cv
import numpy as np

from ..camera.phone import CAMERA_MAT, DIST_COEFFS
from ..config import ROTATION, TRANSLATION, MAP_FACTOR, CHARUCO, DETECT_BOARD
from ..config import BOARD_DEFINITION, ARUCO_TEST_BOARD_DEFINITION, ARUCO_TEST_BOARD_IDS


if DETECT_BOARD == 'final':
    from ..config.boarddef import FINAL_BOARD_DICT, FINAL_BOARD
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


def estimate_pose_and_draw(frame, traj_map, rect_visual):
    corners, ids, _rejected_points = aruco.detectMarkers(frame, dic)
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
                translation_world = ROTATION @ translation + \
                    TRANSLATION.reshape(3, 1)
                # print(f"ROT: {rotation_world}")
                # print(f"TRANS: {translation_world}")
                pos = (int(translation_world[1] * MAP_FACTOR),
                       int(translation_world[0] * MAP_FACTOR))
                traj_map = cv.circle(traj_map, pos, 4, (0, 255, 255), )
                # print(world_map.shape)
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
    return frame, traj_map, rect_visual, rotation, translation, rotation_world, rotation_world
