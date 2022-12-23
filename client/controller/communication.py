import requests
import json
from socket import socket, AF_INET, SOCK_DGRAM
from ..config import BASE_DIR
FRAME_HEADER = "lie-flat device discovery!"

CACHE_PATH = BASE_DIR/'devices.cache.json'


def connect_to_board():
    if CACHE_PATH.is_file():
        with CACHE_PATH.open() as f:
            devices = json.load(f)
        response = requests.post(
            "http://" + devices['board'] + "/init")
        if response.status_code == 200:
            return devices
        raise Exception("Board not initialized!")
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 1234))
    devices = dict()

    while True:
        if 'board' in devices:
            try:
                response = requests.post(
                    "http://" + devices['board'] + "/init")
                if response.status_code == 200:
                    break
            except:
                continue
        else:
            msg, (ip, port) = s.recvfrom(1024)
            lines = msg.decode().splitlines()
            if len(lines) < 3:
                continue
            itr = iter(lines)
            if next(itr) != FRAME_HEADER or next(itr) != "Info":
                # Verify header and verb
                continue
            try:
                for line in itr:
                    ip, name = line.split("=")
                    devices[name] = ip
            except:
                continue
    with CACHE_PATH.open('w') as f:
        json.dump(devices, f)
    return devices


def read_sensors(ip):
    response = requests.get("http://" + ip + "/read")
    if response.status_code == 200:
        return response.json()
    return None


if __name__ == '__main__':
    devices = connect_to_board()
    print(devices)
