import gym
import pybullet as p
from IPython import embed

if __name__ == '__main__':
    env = gym.make("ParkingLot-v0", render=True, mode='1')
    env.reset()
    unwrapped = env.unwrapped
    car = unwrapped.car
    reset = env.reset

    def movement_generator(action):
        def f(t=2):
            for _ in range(t):
                obs = env.step(action)
            else:
                return obs
        return f
    w = movement_generator(0)
    s = movement_generator(1)
    a = movement_generator(2)
    d = movement_generator(3)
    embed(header="You are on your own now. Feel free to explore!")
