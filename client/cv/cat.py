# Meow ~ Meow ~ Meow ~

import cv2 as cv
import numpy as np
from PIL import ImageDraw, Image

from ..camera.phone import PHONE_CAM_WIDTH, PHONE_CAM_HEIGHT
from ..camera.camera import CAR_CAM_WIDTH, CAR_CAM_HEIGHT
from ..config import MAP_LEN_Y, MAP_LEN_X, CHINESE_FONT

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
img_pil = Image.fromarray(TEXT_AREA)
draw = ImageDraw.Draw(img_pil)
draw.text((10, 10),  "关注山威数科班谢谢喵~~~",
          font=CHINESE_FONT, fill=(0xFF, 0x90, 0x1E))
TEXT_AREA = np.array(img_pil)

FONT_SCALE = 1.3
FONT_LINE_WIDTH = 2
CV_FONT = cv.FONT_HERSHEY_DUPLEX

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

    cv.putText(info_area,  "World X : 2423532", (10, 40),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "World Y : 2423532", (10, 80),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "World Z : 2423532", (10, 120),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "World RX: 2423532", (10, 160),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "World RY: 2423532", (10, 200),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "World RZ: 2423532", (10, 240),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "Cam  X : 2423532", (10, 280),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "Cam  Y : 2423532", (10, 320),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "Cam  Z : 2423532", (10, 360),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "Cam  RX: 2423532", (10, 400),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "Cam  RY: 2423532", (10, 440),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "Cam  RZ: 2423532", (10, 480),
               CV_FONT, FONT_SCALE, (255, 0, 255), FONT_LINE_WIDTH, cv.LINE_AA)
    text_area = TEXT_AREA.copy()
    row1 = np.hstack([phone_cam, road_mask, road_perspective])
    row2 = np.hstack([traj, visual, info_area])
    return np.vstack([row1, row2, text_area])
