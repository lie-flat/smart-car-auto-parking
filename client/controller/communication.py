import requests
from socket import socket, AF_INET, SOCK_DGRAM
FRAME_HEADER = "lie-flat device discovery!"


def connect_to_board():
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 1234))
    devices = dict()

    while True:
        if 'board' in devices:
            try:
                response = requests.post(
                    "http://" + devices['board'] + "/init")
                if response.status_code == 200:
                    print("Board initialized!")
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
    return devices


if __name__ == '__main__':
    devices = connect_to_board()
    print(devices)