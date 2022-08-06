import cv2
import numpy as np
from urllib.request import urlopen

CAMERA_BUFFRER_SIZE = 4096


class CameraReader:
    def __init__(self, ip) -> None:
        self.bts = b''
        self.url = f"http://{ip}/capture"

    def read(self):
        while True:
            try:
                req = urlopen(self.url)
                arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
                return cv2.imdecode(arr, -1)
            except Exception as e:
                print("Camera Read Error:" + str(e))
                continue