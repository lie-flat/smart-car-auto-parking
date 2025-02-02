from math import sqrt
import numpy as np
import spatialmath.base as tr

from .common import BASE_DIR
from .positioning import MAP_HEIGHT, MAP_WIDTH

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

TARGET_X = (TARGET_AREA_TOP_LEFT[0] + 1.2)
TARGET_Y = 0.4 * TARGET_AREA_TOP_LEFT[1] + 0.6 * TARGET_AREA_BOTTOM_LEFT[1]

REAL_CAR_SPEED = 55
REAL_CAR_TURN_SPEED = 60

ENVINFO_FILELOCK_PATH = BASE_DIR/"env.shm.lock"
FEEDBACK_FILELOCK_PATH = BASE_DIR/"feedback.shm.lock"

ENVINFO_SHM_NAME = "envinfo"
ENVINFO_DTYPE = np.float32
ENVINFO_SIZE = 6

FEEDBACK_SHM_NAME = "feedback"
FEEDBACK_DTYPE = np.float32
FEEDBACK_SIZE = 6

REAL2SIM = tr.transl(MAP_WIDTH/2, MAP_HEIGHT/2,
                     0) @ tr.trotx(180, unit='deg') @ tr.trotz(90, unit="deg")

SIM_SCALE = 5
