
import cv2 as cv

ip = input("IP address of your phone: ")
vid = cv.VideoCapture(f"http://{ip}:4747/video?640x480")

while True:
    ret, frame = vid.read()
    if not ret:
        raise "Failed to read image!"
    cv.imshow('image', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv.destroyAllWindows()
