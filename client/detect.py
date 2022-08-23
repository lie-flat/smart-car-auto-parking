from ai import DetModel
from camera import CameraReader
from controller import connect_to_board, buzz as buzz_raw
from functools import partial

if __name__ == "__main__":
    model = DetModel()
    devices = connect_to_board()
    buzz = partial(buzz_raw, devices['board'])
    camera = CameraReader(devices['esp32-cam'])
    while True:
        img = camera.read()
        result = model.predict(img)
        print(result)
        buzz(1000,500)
        
