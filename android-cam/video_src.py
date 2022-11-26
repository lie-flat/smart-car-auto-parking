import cv2 as cv
from config import IS_RASPBERRYPI, ip

if ip is None:
    ip = input("IP address of your phone: ")

if IS_RASPBERRYPI:
    vid = cv.VideoCapture(0)
else:
    vid = cv.VideoCapture(f"http://{ip}:4747/video?640x480")
