import logging
import pygame
from ..controller import connect_to_board, buzz, control
from functools import partial

pygame.init()
clock = pygame.time.Clock()
keepPlaying = True

servo = 7.5
motorA = 36
motorB = 0


joysticks = []

for i in range(0, pygame.joystick.get_count()):
    # create an Joystick object in our list
    joysticks.append(pygame.joystick.Joystick(i))
    # initialize them all (-1 means loop forever)
    joysticks[-1].init()


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


logging.basicConfig(
    level=logging.INFO, format='%(asctime)s [%(name)s]%(levelname)s:%(message)s')
log = logging.getLogger("main")
devices = connect_to_board()
log.info(f"Found devices: {devices}")
ip = devices['board']
control = partial(control, ip)
buzz = partial(buzz, ip)
# control = lambda *a, **b: None
# buzz = lambda *a, **b: None
control(servo=7.5, motorA=0, motorB=0)

lt = -1
rt = -1
speed = 0
should_buzz = False
buzz_freq = 3000
buzz_duration = 700


while keepPlaying:
    clock.tick(20)
    for event in pygame.event.get():
        print(event)

        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 2:
                lt = event.value
            elif event.axis == 5:
                rt = event.value
            else:
                continue
        elif event.type == pygame.JOYBUTTONUP:
            if event.button == 5:
                speed += 10
            elif event.button == 4:
                speed -= 10
            elif event.button == 0:
                should_buzz = True
                buzz_freq = 100
            elif event.button == 1:
                should_buzz = True
                buzz_freq = 500
            elif event.button == 2:
                should_buzz = True
                buzz_freq = 1000
            elif event.button == 3:
                should_buzz = True
                buzz_freq = 5000
    # print(f"lt = {lt}, rt={rt}")
    servo = ((rt + 1) - (lt + 1)) * 5 / 2 + 7.5
    servo = clamp(servo, 2.5, 12.5)
    speed = clamp(speed, -100, 100)
    # print(f"Servo = {servo}")
    motorA = 0 if speed < 0 else speed
    motorB = 0 if speed > 0 else speed
    control(servo=servo, motorA=motorA, motorB=motorB)
    if should_buzz:
        should_buzz = False
        buzz(buzz_freq, buzz_duration)
