from controller import connect_to_board, read_sensors, control
from camera import CameraReader
import cv2
import random

def main():
    devices = connect_to_board()
    print(devices)
    print(read_sensors(devices['board']))
    camIP = devices['esp32-cam']
    boardIP = devices['board']
    reader = CameraReader(camIP)
    while True:
        img = reader.read()
        cv2.imshow('img', img)
        servo = random.uniform(2.5,12.5)
        lucky = random.random() > 0.5
        motorA = random.uniform(*((0,70) if lucky else (30,100)))
        motorB = random.uniform(*((30,100) if lucky else (0,70)))
        print(f"Set servo to {servo} and motors to {motorA} and {motorB}")
        control(boardIP, servo, motorA, motorB)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    main()