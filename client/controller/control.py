import requests


def control(ip, servo=None, motorA=None, motorB=None):
    body = {}
    if servo is not None:
        body['servo'] = servo
    if motorA is not None:
        body['motorA'] = motorA
    if motorB is not None:
        body['motorB'] = motorB
    requests.post(f"http://{ip}/cmd", body)
