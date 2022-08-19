import cv2


class CVReader:
    def __init__(self, width, height, device=0):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        while True:
            success, img = self.cap.read()
            if success:
                return img
