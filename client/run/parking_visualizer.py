import cv2 as cv
import logging
import numpy as np
from time import time
from functools import partial
import spatialmath.base as tr

from ..cv import cat, estimate_pose_and_draw
from ..camera import get_phone_video
from ..config import IMG_RESULT_SHAPE, RECORDER, RECORDER_OUTPUT_FILE, FINAL_BOARD_WIDTH, FINAL_BOARD_HEIGHT, MAP_HEIGHT, MAP_WIDTH
from ..config.rl import REAL2SIM
from ..rl.analytics import AnalyticsReader

FONT_SCALE = 1.3
FONT_LINE_WIDTH = 2
CV_FONT = cv.FONT_HERSHEY_DUPLEX
CV_COLOR = (255, 0, 255)

img_result = np.zeros(IMG_RESULT_SHAPE, dtype=np.uint8)
rl_info_area = np.zeros((480, 640, 3), dtype=np.uint8)

analytics_reader = AnalyticsReader()
analytics = {}

"""
Put some info on img_warp
Last Action: Forward/Backward/Left/Right
Last Reward: NaN
Cummulative Reward: NaN
X POS:
Y POS:
X Velocity: 
Y Velocity: 
Z Euler Angle:
Step Counter:
Success: True/False
"""


draw_rl_info = partial(cv.putText, rl_info_area, fontFace=CV_FONT, fontScale=FONT_SCALE,
                       color=CV_COLOR, thickness=FONT_LINE_WIDTH, lineType=cv.LINE_AA)
sim_tr = np.eye(4, dtype=np.float32)
euler = np.array([np.nan, np.nan, np.nan])


def update_rl_info():
    analytics_reader.read_to_dict(analytics)
    rl_info_area.fill(255)
    draw_rl_info(text=f"Last Action: {analytics['last_action']}", org=(10, 40))
    draw_rl_info(text=f"Last Reward: {analytics['last_reward']}", org=(10, 80))
    draw_rl_info(
        text=f"Cummulative Reward: {analytics['cummulative_reward']}", org=(10, 120))
    draw_rl_info(
        text=f"Step Counter: {analytics['step_counter']}", org=(10, 160))
    draw_rl_info(text=f"Success: {analytics['success']}", org=(10, 200))
    draw_rl_info(text=f"X Pos: {sim_tr[0,3].item():.8f}", org=(10, 240))
    draw_rl_info(text=f"Y Pos: {sim_tr[1,3].item():.8f}", org=(10, 280))
    draw_rl_info(text=f"X Euler: {euler[0].item():.8f}", org=(10, 320))
    draw_rl_info(text=f"Y Euler: {euler[1].item():.8f}", org=(10, 360))
    draw_rl_info(text=f"Z Euler: {euler[2].item():.8f}", org=(10, 400))


if RECORDER == 'video':
    fourcc = cv.VideoWriter_fourcc(*'MP4V')
    writer = cv.VideoWriter(RECORDER_OUTPUT_FILE, fourcc, 18.0, (1920, 1080))
else:
    writer = None


def main():
    global img_result, rl_info_area, sim_tr, euler
    world_tr = np.ndarray((4, 4), dtype=np.float32)
    world_tr[3, :] = [0, 0, 0, 1]
    cv.namedWindow("frame", cv.WINDOW_NORMAL)
    cv.setWindowProperty("frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    vid = get_phone_video()
    log = logging.getLogger()
    # used to record the time when we processed last frame
    prev_frame_time = 0
    # used to record the time at which we processed current frame
    new_frame_time = 0
    while True:
        ret, frame = vid.read()
        if not ret:
            raise Exception("Failed to read from phone.")
        frame, rotation, translation, rotation_world, translation_world \
            = estimate_pose_and_draw(frame)
        if translation_world is not None:
            world_tr[:3, :3] = rotation_world
            world_tr[3, 3] = 1
            # Calculate the center of aruco board
            cost = rotation_world[0, 0]
            sint = rotation_world[1, 0]
            p0x = translation_world[1].item()
            p0y = translation_world[0].item()
            p1x = p0x + FINAL_BOARD_WIDTH * cost
            p1y = p0y - FINAL_BOARD_WIDTH * sint
            p3x = p0x + FINAL_BOARD_HEIGHT * sint
            p3y = p0y + FINAL_BOARD_HEIGHT * cost
            center_x = (p1x + p3x)/2
            center_y = (p1y + p3y)/2
            # print(center_x, center_y)
            world_tr[:2, 3] = [center_x, center_y]
            print(world_tr)
            sim_tr = world_tr @ REAL2SIM 
            euler = tr.tr2eul(sim_tr)
        update_rl_info()
        new_frame_time = time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        all_concat = cat(frame, img_result, rl_info_area, translation_world,
                         rotation_world, translation, rotation, fps)
        cv.imshow("frame", all_concat)
        if writer is not None:
            writer.write(all_concat)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    if writer is not None:
        writer.release()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s#%(levelname)s:%(message)s')
    try:
        main()
    finally:
        pass
