import numpy as np
import cv2 as cv

OFFSET_X = 0.45
OFFSET_Y = 0.65
OFFSET_Z = -1.05

TRANS = np.array([[1, 0, 0, OFFSET_X], [0, 1, 0, OFFSET_Y],
                 [0, 0, 1, OFFSET_Z], [0, 0, 0, 1]])

TRANSLATION = np.array([OFFSET_X, OFFSET_Y, OFFSET_Z], dtype="float32")

ROTATION = np.array([
    [0, 1, 0],
    [-1, 0, 0],
    [0, 0, 1]
], dtype="float32")


MAP_LEN_X = int(1015 / 2)
MAP_LEN_Y = int(1210 / 2)
MAP_FACTOR = 1e3 / 2