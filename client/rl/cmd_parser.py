import argparse
from datetime import datetime
import math
from os import path
import numpy as np

from ..config.rl import LOG_DIR


def build_parser(f=lambda _: None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='dqn',
                        help='the model to use', choices=['dqn', 'ppo'])
    parser.add_argument('--env', type=str, default="ParkingLot-v0",
                        help='name of the environment to run')
    parser.add_argument('--render', type=bool, default=False,
                        help='render the environment', action=argparse.BooleanOptionalAction)
    parser.add_argument('--presentation', type=bool, default=False,
                        help='presentation mode.', action=argparse.BooleanOptionalAction)
    parser.add_argument('--seed', type=int, default=0,
                        help='seed for RNG')
    parser.add_argument('--log-dir', type=str,
                        default='', help='log dir')
    parser.add_argument('--model-path', type=str,
                        default='', help='model load/store path')
    parser.add_argument('--car', type=str, default='husky',
                        help='car model to use')
    parser.add_argument('--car-scale', type=float, default=1.1,
                        help='scaling of the car model')
    parser.add_argument('--init-x', type=float, default=-1.5,
                        help='initial x position')
    parser.add_argument('--init-y', type=float, default=1.45,
                        help='initial y position')
    parser.add_argument('--init-theta', type=str, default="np.pi",
                        help='initial theta, python expression evaluation is supported')
    parser.add_argument('--real', type=bool, default=False,
                        help='Real world or not', action=argparse.BooleanOptionalAction)
    parser.add_argument('--wall', type=bool, default=True,
                        help='Add walls or not', action=argparse.BooleanOptionalAction)
    parser.add_argument('--eval-episodes', type=int,
                        default=int(10), help='total episodes to eval')
    f(parser)
    return parser


def grab_args(parser):
    args = parser.parse_args()
    args.init_theta = eval(args.init_theta, {"np": np, "math": math})
    time = datetime.strftime(datetime.now(), '%m%d_%H%M')
    default_log_dir = LOG_DIR / \
        f'{args.model}_{args.init_x:.2},{args.init_y:.2},{args.init_theta:.2}_{time}'
    if not args.log_dir:
        args.log_dir = str(default_log_dir)
    if not args.model_path:
        args.model_path = path.join(args.log_dir, "final.zip")
    return args
