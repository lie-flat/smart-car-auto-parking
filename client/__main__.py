from functools import partial
from multiprocessing.shared_memory import SharedMemory
import cv2 as cv
import logging
import numpy as np


from .cv import cat, estimate_pose_and_draw
from .camera import get_phone_video
from .camera.camera import CAR_CAM_HEIGHT, CAR_CAM_WIDTH
from .config import MAP_LEN_X, MAP_LEN_Y,\
    SHM_NP_DTYPE, SHM_IMG_RESULT_NAME, SHM_IMG_WARP_NAME, \
    IMG_RESULT_SHAPE, IMG_WARP_SHAPE


img_result_shm = SharedMemory(name=SHM_IMG_RESULT_NAME)
img_result = np.ndarray(IMG_RESULT_SHAPE, dtype=SHM_NP_DTYPE,
                        buffer=img_result_shm.buf)
img_warp_shm = SharedMemory(name=SHM_IMG_WARP_NAME)
img_warp = np.ndarray(IMG_WARP_SHAPE, dtype=SHM_NP_DTYPE,
                      buffer=img_warp_shm.buf)
traj_map = 255 * np.ones((MAP_LEN_X, MAP_LEN_Y, 3), dtype="uint8")
rect_visual = 255 * np.ones((MAP_LEN_X, MAP_LEN_Y, 3), dtype="uint8")


def main():
    global traj_map, rect_visual, img_result, img_warp
    cv.namedWindow("frame", cv.WINDOW_NORMAL)
    cv.setWindowProperty("frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    vid = get_phone_video()
    log = logging.getLogger()
    while True:
        ret, frame = vid.read()
        if not ret:
            raise Exception("Failed to read from phone.")
        frame, traj_map, rect_visual, \
            rotation, translation,\
            rotation_world, translation_world = estimate_pose_and_draw(
                frame, traj_map, rect_visual)
        log.info("Pose estimated")
        all_concat = cat(frame, img_result, img_warp, traj_map, rect_visual,
                         translation_world, rotation_world, translation, rotation)
        log.info("Concatnated")
        cv.imshow("frame", all_concat)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s#%(levelname)s:%(message)s')
    try:
        main()
    finally:
        img_result_shm.close()
        img_warp_shm.close()
