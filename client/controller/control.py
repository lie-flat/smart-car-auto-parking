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


def act(ip, servo=7.5, a=0, b=0, duration=0):
    body = {
        "servo": servo,
        "motorA": a,
        "motorB": b,
        "duration": duration
    }
    requests.post(f"http://{ip}/act", body)


def buzz(ip, freq=3000, duration=500):
    requests.post(f"http://{ip}/buzz", {
        'freq': freq, 'duration': duration
    })
