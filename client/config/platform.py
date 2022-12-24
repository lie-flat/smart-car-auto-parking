import os

user = os.getlogin()

IS_RASPBERRYPI = user == 'pi'

print(f"Running on {'Raspberry Pi' if IS_RASPBERRYPI else 'PC'}")

ip = '192.168.12.111'