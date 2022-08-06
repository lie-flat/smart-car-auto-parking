import requests

def control(ip,servo=7.5, motorA=0, motorB=0):
    requests.post(f"http://{ip}/cmd", {"servo": servo, "motorA": motorA, "motorB": motorB})
