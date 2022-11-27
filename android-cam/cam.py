
import cv2 as cv
from video_src import vid

while True:
    ret, frame = vid.read()
    if not ret:
        raise Exception("Read failure!")
    cv.circle(frame, (320, 240), 5, (255,0,0), 5)
    cv.imshow('image', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv.destroyAllWindows()
