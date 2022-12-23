import gym
import pybullet as p
from IPython import embed
from .cmd_parser import build_parser, grab_args
from .impl import make_env
from time import sleep

if __name__ == '__main__':
    parser = build_parser()
    args = grab_args(parser)
    args.render = True
    env = make_env(args)
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
