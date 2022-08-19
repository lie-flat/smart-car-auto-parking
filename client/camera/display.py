import cv2
import sys
from camera import CameraReader
from cv import CVReader


def main():
    if len(sys.argv) < 2:
        print("ERROR: no args!")
        print("USAGE: display.py pi/<YOUR-ESP32CAM-IP>")
        return
    if sys.argv[1] == 'pi':
        reader = CVReader(480, 320)
    else:
        reader = CameraReader(sys.argv[1])
    while True:
        img = reader.read()
        cv2.imshow("img", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    main()
