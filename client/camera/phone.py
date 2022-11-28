import cv2 as cv
from ..config import IS_RASPBERRYPI, ip

def get_phone_video():
    global ip
    if IS_RASPBERRYPI:
        return cv.VideoCapture(0)
    else:
        if ip is None:
            ip = input("IP address of your phone: ")
        return cv.VideoCapture(f"http://{ip}:4747/video?640x480")