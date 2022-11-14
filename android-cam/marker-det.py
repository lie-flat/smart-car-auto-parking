import cv2 as cv

ip = input("IP address of your phone: ")
vid = cv.VideoCapture(f"http://{ip}:4747/video?640x480")
dic = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)

while True:
    ret, frame = vid.read()
    if not ret:
        raise Exception("Failed to read image!")
    corners, ids, rejected_points = cv.aruco.detectMarkers(frame, dic)
    if ids is not None and len(ids) > 0:
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
        print( ids)
    cv.imshow('image', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv.destroyAllWindows()
