import cv2
import numpy as np
from urllib.request import urlopen

CAMERA_BUFFRER_SIZE = 4096


class CameraReader:
    def __init__(self, ip) -> None:
        self.bts = b''
        self.url = f"http://{ip}:81/stream"
        self.stream = urlopen(self.url)

    def read(self):
        while True:
            try:
                self.bts += self.stream.read(CAMERA_BUFFRER_SIZE)
                jpghead = self.bts.find(b'\xff\xd8')
                jpgend = self.bts.find(b'\xff\xd9')
                if jpghead > -1 and jpgend > -1:
                    jpg = self.bts[jpghead:jpgend+2]
                    self.bts = self.bts[jpgend+2:]
                    return cv2.imdecode(np.frombuffer(
                        jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            except Exception as e:
                print("Error:" + str(e))
                self.bts = b''
                self.stream = urlopen(self.url)
                continue
