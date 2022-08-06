import sys
import cv2
import numpy as np
from urllib.request import urlopen
from datetime import datetime

ip = sys.argv[1]
url = f"http://{ip}/capture"
while True:
    req = urlopen(url)
    arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
    img = cv2.imdecode(arr, -1)
    cv2.imshow("Image", img)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('s'):
        # Press s to save
        cv2.imwrite(datetime.strftime(datetime.now(),"%Y-%m-%d-%H-%M-%S") + ".jpg", img)
    if key & 0xFF == ord('q'):
        break
