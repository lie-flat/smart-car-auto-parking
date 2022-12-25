import cv2 as cv
import logging
import numpy as np
from time import time
from functools import partial
from filelock import FileLock
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--follow', default=False, action=argparse.BooleanOptionalAction,
                    help="add this option to enable follow mode")
args = parser.parse_args()


# fmt: off
from ..config import context, IMG_RESULT_SHAPE, RECORDER, RECORDER_OUTPUT_FILE, FINAL_BOARD_WIDTH, FINAL_BOARD_HEIGHT
context['mode'] = 'parking-follow' if args.follow else 'parking'


from ..cv import cat, estimate_pose_and_draw
from ..camera import get_phone_video
from ..utils import create_shared_memory_nparray
from ..config.rl import REAL2SIM, SIM_SCALE, FEEDBACK_DTYPE, FEEDBACK_SIZE, FEEDBACK_SHM_NAME, FEEDBACK_FILELOCK_PATH
from ..rl.analytics import AnalyticsReader

# fmt: on


FONT_SCALE = 1.25
FONT_LINE_WIDTH = 2
CV_FONT = cv.FONT_HERSHEY_DUPLEX
CV_COLOR = (0x9a, 0x97, 0x43)

img_result = np.zeros(IMG_RESULT_SHAPE, dtype=np.uint8)
rl_info_area = np.zeros((480, 640, 3), dtype=np.uint8)

analytics_reader = AnalyticsReader()
analytics = {}

draw_rl_info_with_color = partial(cv.putText, rl_info_area, fontFace=CV_FONT, fontScale=FONT_SCALE,
                                  thickness=FONT_LINE_WIDTH, lineType=cv.LINE_AA)
draw_rl_info = partial(draw_rl_info_with_color, color=CV_COLOR)

feedback_shm = create_shared_memory_nparray(np.zeros(
    FEEDBACK_SIZE, dtype=FEEDBACK_DTYPE), FEEDBACK_SHM_NAME, FEEDBACK_DTYPE)
feedback_arr = np.ndarray(
    FEEDBACK_SIZE, dtype=FEEDBACK_DTYPE, buffer=feedback_shm.buf)
feedback_lock = FileLock(FEEDBACK_FILELOCK_PATH)


# fmt: off
def update_rl_info(pos, velocity, z_rotation, z_cos_t, z_sin_t):
    analytics_reader.read_to_dict(analytics)
    rl_info_area.fill(255)
    draw_rl_info(text=f"  Last Action  : {analytics['last_action']}", org=(10, 35))
    draw_rl_info(text=f"  Last Reward : {analytics['last_reward']}", org=(10, 70))
    draw_rl_info(text=f" Cum Reward  : {analytics['cummulative_reward']}", org=(10, 105))
    draw_rl_info(text=f" Step  Counter : {analytics['step_counter']}", org=(10, 140))
    draw_rl_info(text=f" Distance  : {analytics['distance']}", org=(10, 175))
    draw_rl_info_with_color(text=f"    > Observation Data <", org=(10, 210), color=(255,255,0))
    draw_rl_info(text=f"  X Pos   : {pos[0]:.8f}", org=(10, 245))
    draw_rl_info(text=f"  Y Pos   : {pos[1]:.8f}", org=(10, 280))
    draw_rl_info(text=f"X Velocity : {velocity[0]:.8f}", org=(10, 315))
    draw_rl_info(text=f"Y Velocity : {velocity[1]:.8f}", org=(10, 350))
    draw_rl_info(text=f"Z Rotation: {z_rotation:.8f}", org=(10, 385))
    draw_rl_info(text=f"Cos(Rz)={z_cos_t:.8f}", org=(10, 430))
    draw_rl_info(text=f"Sin(Rz)={z_sin_t:.8f}", org=(10, 470))
# fmt: on


if RECORDER == 'video':
    fourcc = cv.VideoWriter_fourcc(*'MP4V')
    writer = cv.VideoWriter(RECORDER_OUTPUT_FILE, fourcc, 18.0, (1920, 1080))
else:
    writer = None


def main():
    global img_result, rl_info_area, last_pos, feedback_lock, feedback_arr
    last_pos_measure_time = None
    sim_tr = np.eye(4, dtype=np.float32)
    pos = np.array([np.nan, np.nan])
    z_rotation = np.nan
    z_sin_t = np.nan
    z_cos_t = np.nan
    velocity = np.array([np.nan, np.nan])
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
        if translation_world is not None and context['mode'] == 'parking':
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
            world_tr[:2, 3] = [center_y, center_x]  # This is not a mistake
            last_pos = pos
            sim_tr = REAL2SIM @ world_tr
            time_now = time()
            if last_pos_measure_time is None:
                last_pos_measure_time = time()
            delta_t = time_now - last_pos_measure_time
            pos = SIM_SCALE * sim_tr[:2, 3]
            velocity = (pos - last_pos) / delta_t
            last_pos_measure_time = time_now
            z_rotation = np.pi - abs(np.arctan2(sim_tr[1, 0], sim_tr[0, 0]))
            z_cos_t = np.cos(z_rotation)
            z_sin_t = np.sin(z_rotation)
            with feedback_lock:
                # share observation via shared memory
                feedback_arr[:2] = pos
                feedback_arr[2:4] = velocity
                feedback_arr[4:] = [z_cos_t, z_sin_t]

        update_rl_info(pos, velocity, z_rotation, z_cos_t, z_sin_t)
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
        analytics_reader.shm.unlink()
