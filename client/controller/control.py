import requests

def control(ip,servo, motorA, motorB):
    requests.post(f"http://{ip}/cmd", {"servo": servo, "motorA": motorA, "motorB": motorB})
