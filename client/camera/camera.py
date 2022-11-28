import cv2 as cv
import numpy as np
from urllib.request import urlopen

import logging

CAMERA_BUFFRER_SIZE = 4096

CAR_CAM_WIDTH = 320
CAR_CAM_HEIGHT = 240

class CameraReader:
    def __init__(self, ip) -> None:
        self.log = logging.getLogger()
        self.url = f"http://{ip}/capture"

    def read(self):
        while True:
            try:
                req = urlopen(self.url)
                arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
                return cv.imdecode(arr, -1)
            except Exception as e:
                self.log.error("Read Error:" + str(e))
                continue



