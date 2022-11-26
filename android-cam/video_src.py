import cv2 as cv

ip = input("IP address of your phone: ")
vid = cv.VideoCapture(f"http://{ip}:4747/video?640x480")