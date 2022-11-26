import numpy as np

SPACE_TOP = 90
MARKER_SIZE = 20
SPACE_BOTTOM = 105
BOTTOM_X_OFFSET = 130  # including marker
Z_TOP = 0
BOTTOM_Z_OFFSET = 2
BOTTOM_MARKER_Y_OFFSET = (SPACE_BOTTOM - SPACE_TOP) / 2

BOARD_DEFINITION = np.array([
    [
        # Marker Top Left
        [0, 0, Z_TOP],
        [0, MARKER_SIZE, Z_TOP],
        [MARKER_SIZE, MARKER_SIZE, Z_TOP],
        [MARKER_SIZE, 0, Z_TOP],
    ], [
        # Marker Top Right
        [0, MARKER_SIZE + SPACE_TOP, Z_TOP],
        [0, MARKER_SIZE + SPACE_TOP + MARKER_SIZE, Z_TOP],
        [MARKER_SIZE, MARKER_SIZE + SPACE_TOP + MARKER_SIZE, Z_TOP],
        [MARKER_SIZE, MARKER_SIZE + SPACE_TOP, Z_TOP],
    ], [
        # Marker Bottom Right
        [BOTTOM_X_OFFSET, BOTTOM_MARKER_Y_OFFSET + \
            MARKER_SIZE + SPACE_TOP, Z_TOP + BOTTOM_Z_OFFSET],
        [BOTTOM_X_OFFSET, BOTTOM_MARKER_Y_OFFSET + \
            MARKER_SIZE + SPACE_TOP + MARKER_SIZE, Z_TOP + BOTTOM_Z_OFFSET],
        [BOTTOM_X_OFFSET + MARKER_SIZE,  BOTTOM_MARKER_Y_OFFSET + \
            MARKER_SIZE + SPACE_TOP + MARKER_SIZE, Z_TOP + BOTTOM_Z_OFFSET],
        [BOTTOM_X_OFFSET + MARKER_SIZE, BOTTOM_MARKER_Y_OFFSET + \
            MARKER_SIZE + SPACE_TOP, Z_TOP + BOTTOM_Z_OFFSET],
    ], [
        # Marker Bottom Left
        [BOTTOM_X_OFFSET, -BOTTOM_MARKER_Y_OFFSET, Z_TOP + BOTTOM_Z_OFFSET],
        [BOTTOM_X_OFFSET, -BOTTOM_MARKER_Y_OFFSET, Z_TOP + BOTTOM_Z_OFFSET],
        [BOTTOM_X_OFFSET + MARKER_SIZE, - BOTTOM_MARKER_Y_OFFSET + \
            MARKER_SIZE, Z_TOP + BOTTOM_Z_OFFSET],
        [BOTTOM_X_OFFSET + MARKER_SIZE, - BOTTOM_MARKER_Y_OFFSET + \
            MARKER_SIZE, Z_TOP + BOTTOM_Z_OFFSET],
    ]
], dtype="float32")

#####################################


ARUCO_TEST_BOARD_MARKER_SIZE = 25
ARUCO_TEST_BOARD_PADDING = 10
ARUCO_TEST_BOARD_Z = 0
ARUCO_TEST_BOARD_SIZE = (5,4)
ARUCO_TEST_BOARD_IDS = np.array([1008, 92, 410, 35]*ARUCO_TEST_BOARD_SIZE[0])


def calculate_test_board_marker_pos(row, col):
    """
    row, col counts from zero!
    """
    top_left = [(ARUCO_TEST_BOARD_MARKER_SIZE + ARUCO_TEST_BOARD_PADDING) * row,
                (ARUCO_TEST_BOARD_MARKER_SIZE + ARUCO_TEST_BOARD_PADDING) * col, ARUCO_TEST_BOARD_Z]
    return [
        top_left,
        [top_left[0], top_left[1] + ARUCO_TEST_BOARD_MARKER_SIZE, ARUCO_TEST_BOARD_Z],
        [top_left[0] + ARUCO_TEST_BOARD_MARKER_SIZE, top_left[1] +
            ARUCO_TEST_BOARD_MARKER_SIZE, ARUCO_TEST_BOARD_Z],
        [top_left[0] + ARUCO_TEST_BOARD_MARKER_SIZE, top_left[1], ARUCO_TEST_BOARD_Z]
    ]


ARUCO_TEST_BOARD = [calculate_test_board_marker_pos(
    i, j) for i in range(ARUCO_TEST_BOARD_SIZE[0]) for j in range(ARUCO_TEST_BOARD_SIZE[1])]

ARUCO_TEST_BOARD_DEFINITION = np.array(ARUCO_TEST_BOARD, dtype="float32")
