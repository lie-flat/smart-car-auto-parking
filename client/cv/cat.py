# Meow ~ Meow ~ Meow ~

import cv2 as cv
import numpy as np
from PIL import ImageDraw, Image

from ..camera.phone import PHONE_CAM_WIDTH, PHONE_CAM_HEIGHT
from ..camera.camera import CAR_CAM_WIDTH, CAR_CAM_HEIGHT
from ..config import MAP_LEN_Y, MAP_LEN_X, CHINESE_FONT

VIDEO_WIDTH = 1920
VIDEO_HEIGHT = 1080


SEPARATOR_WIDTH = 10
SEPARATOR_HEIGHT = MAP_LEN_X
SEPARATOR = np.ones((SEPARATOR_HEIGHT, SEPARATOR_WIDTH, 3),
                    dtype=np.uint8) * 50

INFO_AREA_HEIGHT = MAP_LEN_X
INFO_AREA_WIDTH = VIDEO_WIDTH - 2 * MAP_LEN_Y - 2 * SEPARATOR_WIDTH
INFO_AREA = np.ones(
    (INFO_AREA_HEIGHT, INFO_AREA_WIDTH, 3), dtype=np.uint8) * 255

TEXT_AREA_HEIGHT = VIDEO_HEIGHT - PHONE_CAM_HEIGHT - MAP_LEN_X
TEXT_AREA_WIDTH = VIDEO_WIDTH

TEXT_AREA = np.ones((TEXT_AREA_HEIGHT, TEXT_AREA_WIDTH, 3),
                    dtype=np.uint8) * 255
img_pil = Image.fromarray(TEXT_AREA)
draw = ImageDraw.Draw(img_pil)
draw.text((10, -3),  "求个 Star, 谢谢喵~: https://github.com/lie-flat/smart-car-auto-parking",
          font=CHINESE_FONT, fill=(0xFF, 0x90, 0x1E))
draw.text((1260, -3),  "非常感谢得意黑 SmileySans 这款开源字体",
          font=CHINESE_FONT, fill=(0x75, 0x7A, 0x0B))
draw.text((10, 45),  "*: 因为 ESP32 CAM 网络延迟问题，小车摄像头的画面有时会有不确定的延迟（一般在 1s 左右）",
          font=CHINESE_FONT, fill=(0x4B, 0x4B, 0xE5))
draw.text((1500, 45),  "山东大学（威海）,数科班",
          font=CHINESE_FONT, fill=(0xC5, 0xFF, 0x00))
TEXT_AREA = np.array(img_pil)


FONT_SCALE = 1.3
FONT_LINE_WIDTH = 2
CV_FONT = cv.FONT_HERSHEY_DUPLEX
CV_COLOR = (255, 0, 255)

PLACEHOLDER = np.array([np.NaN, np.NaN, np.NaN])


def null_coalesce(val, fallback):
    return val if val is not None else fallback


def cat(phone_cam, road_mask, road_perspective, traj, visual, world_trans, world_rot, cam_trans, cam_rot, fps):
    """
            640           640               640
         +------------+-------------+-------------------+
      4  | marker det | road mask   | road(perspective) |
      8  | i:480x640  | i:240x320   | i:240x320x1       |
      0  | 480x640    | 480x640     | 480x640           |
         +------------+-+------------+-+----------------+
      5  | trajectory |S|rect visual |S|info display    |
      0  | i: 507x605 |1|i: 507x605  |1|xyz             |
      7  | o: 507x605 |0|o: 507x605  |0|rotation:       |
         +------------+-+------------+-+-----------------+
      93 | text                                         |
         +----------------------------------------------+
    """
    info_area = INFO_AREA.copy()
    # Resize inputs
    road_mask = cv.resize(road_mask, (PHONE_CAM_WIDTH, PHONE_CAM_HEIGHT))
    road_perspective = cv.cvtColor(road_perspective, cv.COLOR_GRAY2BGR)
    road_perspective = cv.resize(
        road_perspective, (PHONE_CAM_WIDTH, PHONE_CAM_HEIGHT))
    # Null coalescing
    world_trans = null_coalesce(world_trans, PLACEHOLDER)
    cam_trans = null_coalesce(cam_trans, PLACEHOLDER)
    world_rot = null_coalesce(world_rot, PLACEHOLDER)
    cam_rot = null_coalesce(cam_rot, PLACEHOLDER)
    # Convert mats to vecs
    if world_rot.size > 3:
        world_rot, _ = cv.Rodrigues(world_rot)
    # get numbers from vecs
    world_x = world_trans[0].item()
    world_y = world_trans[1].item()
    world_z = world_trans[2].item()
    cam_x = cam_trans[0].item()
    cam_y = cam_trans[1].item()
    cam_z = cam_trans[2].item()
    cam_rx = cam_rot[0].item()
    cam_ry = cam_rot[1].item()
    cam_rz = cam_rot[2].item()
    world_rx = world_rot[0].item()
    world_ry = world_rot[1].item()
    world_rz = world_rot[2].item()
    cv.putText(info_area,  f"World X : {world_x:.8}", (10, 40),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"World Y : {world_y:.8}", (10, 80),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"World Z : {world_z:.8}", (10, 120),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"World RX: {world_rx:.8}", (10, 160),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"World RY: {world_ry:.8}", (10, 200),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"World RZ: {world_rz:.8}", (10, 240),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"Cam  X : {cam_x:.8}", (10, 280),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"Cam  Y : {cam_y:.8}", (10, 320),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"Cam  Z : {cam_z:.8}", (10, 360),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"Cam  RX: {cam_rx:.8}", (10, 400),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"Cam  RY: {cam_ry:.8}", (10, 440),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"Cam  RZ: {cam_rz:.8}", (10, 480),
               CV_FONT, FONT_SCALE, CV_COLOR, FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  "FPS", (550, 40),
               CV_FONT, FONT_SCALE, (0, 255, 0), FONT_LINE_WIDTH, cv.LINE_AA)
    cv.putText(info_area,  f"{fps:.2f}", (550, 80),
               CV_FONT, FONT_SCALE, (0, 255, 0), FONT_LINE_WIDTH, cv.LINE_AA)
    row1 = np.hstack([phone_cam, road_mask, road_perspective])
    row2 = np.hstack([traj, SEPARATOR, visual, SEPARATOR, info_area])
    return np.vstack([row1, row2, TEXT_AREA])
