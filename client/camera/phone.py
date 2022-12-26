from ..config import IS_RASPBERRYPI
import numpy as np
import cv2 as cv
from ..config import IS_RASPBERRYPI, ip, PHONE_CAM_MODE, IRIUN_CAM_ID

PHONE_CAM_WIDTH = 640
PHONE_CAM_HEIGHT = 480


def get_phone_video():
    global ip
    match (PHONE_CAM_MODE, IS_RASPBERRYPI):
        case (_, True):
            return cv.VideoCapture(0)
        case ('droidcam', False):
            if ip is None:
                ip = input("IP address of your phone: ")
            return cv.VideoCapture(f"http://{ip}:4747/video?640x480")
        case ('iriun', False):
            return cv.VideoCapture(IRIUN_CAM_ID)
        case _:
            raise NotImplementedError


if IS_RASPBERRYPI:
    CAMERA_MAT = np.array([[621.98474404,   0., 325.83155255],
                           [0., 622.99381138, 242.90768789],
                           [0.,   0.,   1.]], dtype="float32")
    DIST_COEFFS = np.array(
        [[0.18640865, -0.83050602, -0.00315513,  0.00577651,  0.91716078]], dtype="float32")
else:
    CAMERA_MAT = np.array([[470.75829554,   0., 320.85206853],
                           [0., 472.51686419, 242.5783875],
                           [0.,   0.,   1.]], dtype="float32")
    DIST_COEFFS = np.array(
        [[0.04556832, -0.10639779,  0.00078644,  0.00124518,  0.08526551]], dtype="float32")
