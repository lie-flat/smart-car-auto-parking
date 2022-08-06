# Adapted from https://gist.github.com/youjunjer/79e5dad5f47ee5757fcb9d401a95e76b
import cv2
import numpy as np
from urllib.request import urlopen
import os
import datetime
import time
import sys

# change to your ESP32-CAM ip
url = f"http://{sys.argv[1]}:81/stream"
CAMERA_BUFFRER_SIZE = 4096
stream = urlopen(url)
bts = b''
i = 0

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
            # img=cv2.flip(img,0) #>0:垂直翻轉, 0:水平翻轉, <0:垂直水平翻轉
            # h,w=img.shape[:2]
            #print('影像大小 高:' + str(h) + '寬：' + str(w))
            # img = cv2.resize(img, (640, 480))
            cv2.imshow("a", img)
        k = cv2.waitKey(1)
    except Exception as e:
        print("Error:" + str(e))
        bts = b''
        stream = urlopen(url)
        continue

    k = cv2.waitKey(1)
    # 按a拍照存檔
    if k & 0xFF == ord('a'):
        cv2.imwrite(str(i) + ".jpg", img)
        i = i+1
    # 按q離開
    if k & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
