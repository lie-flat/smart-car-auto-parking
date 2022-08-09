from os import path
from datetime import datetime
import cv2
from .camera import CameraReader


if __name__ == '__main__':
    cam = CameraReader(input("IP: "))
    folder = path.join(path.dirname(__file__), '..', '..', 'train_data')
    while True:
        img = cam.read()
        cv2.imwrite(datetime.strftime(datetime.now(),"%Y-%m-%d-%H-%M-%S.%f")[:-3] + ".jpg", img)