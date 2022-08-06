import cv2
import numpy as np
from urllib.request import urlopen

CAMERA_BUFFRER_SIZE = 4096


def camera_loop(ip, cb):
    bts = b''
    url = f"http://{ip}:81/stream"
    stream = urlopen(url)
    while True:
        try:
            bts += stream.read(CAMERA_BUFFRER_SIZE)
            jpghead = bts.find(b'\xff\xd8')
            jpgend = bts.find(b'\xff\xd9')
            if jpghead > -1 and jpgend > -1:
                jpg = bts[jpghead:jpgend+2]
                bts = bts[jpgend+2:]
                img = cv2.imdecode(np.frombuffer(
                    jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                cb(img)
        except Exception as e:
            print("Error:" + str(e))
            bts = b''
            stream = urlopen(url)
            continue
