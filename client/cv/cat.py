# Meow ~ Meow ~ Meow ~

import cv2 as cv
import numpy as np

from ..camera.phone import PHONE_CAM_WIDTH, PHONE_CAM_HEIGHT
from ..camera.camera import CAR_CAM_WIDTH, CAR_CAM_HEIGHT
from ..config import MAP_LEN_Y, MAP_LEN_X

VIDEO_WIDTH = 1920
VIDEO_HEIGHT = 1080

INFO_AREA_HEIGHT = MAP_LEN_X
INFO_AREA_WIDTH = VIDEO_WIDTH - 2 * MAP_LEN_Y

INFO_AREA = np.ones(
    (INFO_AREA_HEIGHT, INFO_AREA_WIDTH, 3), dtype=np.uint8) * 255

TEXT_AREA_HEIGHT = VIDEO_HEIGHT - PHONE_CAM_HEIGHT - MAP_LEN_X
TEXT_AREA_WIDTH = VIDEO_WIDTH

TEXT_AREA = np.ones((TEXT_AREA_HEIGHT, TEXT_AREA_WIDTH, 3),
                    dtype=np.uint8) * 255


def cat(phone_cam, road_mask, road_perspective, traj, visual, rotation, translation, text):
    """
            640           640               640
         +------------+-------------+-------------------+
      4  | marker det | road mask   | road(perspective) |
      8  | i:240x320  | i:240x320   | i:480x640         |
      0  | 480x640    | 480x640     | 480x640           |
         +------------+-------------+-------------------+
      5  | trajectory | rect visual | info display      |
      0  | i: 507x605 | i: 507x605  | x:...,y:...,z:... |
      7  | o: 507x605 | o: 507x605  | rotation:         |
         +------------+-------------+-------------------+
      93 | text                                         |
         +----------------------------------------------+
    """
    info_area = INFO_AREA.copy()
    text_area = TEXT_AREA.copy()
    row1 = np.hstack([phone_cam, road_mask, road_perspective])
    row2 = np.hstack([traj, visual, info_area])
    np.vsplit([row1, row2, text_area])
