from math import sqrt

from .common import BASE_DIR

LOG_DIR = BASE_DIR/'logs'

MAP_LEN_X = 12.10 / 2
MAP_LEN_Y = 10.15 / 2

MAP_HALF_X = MAP_LEN_X / 2
MAP_HALF_Y = MAP_LEN_Y / 2

DRAWING_Z = 0.02

TARGET_AREA_TOP_LEFT = [45.7/20 - MAP_HALF_X, MAP_HALF_Y - 28.6/20, DRAWING_Z]
TARGET_AREA_TOP_RIGHT = [TARGET_AREA_TOP_LEFT[0] +
                         24/20, TARGET_AREA_TOP_LEFT[1], DRAWING_Z]
TARGET_AREA_BOTTOM_LEFT = [TARGET_AREA_TOP_LEFT[0] + 31/20/sqrt(3),
                           TARGET_AREA_TOP_LEFT[1] - 31/20, DRAWING_Z]
TARGET_AREA_BOTTOM_RIGHT = [
    TARGET_AREA_BOTTOM_LEFT[0] + 24/20, TARGET_AREA_BOTTOM_LEFT[1], DRAWING_Z]

TARGET_X = (TARGET_AREA_TOP_LEFT[0] + 1)
TARGET_Y = (TARGET_AREA_TOP_LEFT[1] + TARGET_AREA_BOTTOM_LEFT[1])/2
