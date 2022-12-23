import argparse
from datetime import datetime

from ..config.rl import LOG_DIR


def build_parser(f=lambda _: None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='dqn',
                        help='the model to use', choices=['dqn', 'ppo'])
    parser.add_argument('--env', type=str, default="ParkingLot-v0",
                        help='name of the environment to run')
    parser.add_argument('--render', type=bool, default=False,
                        help='render the environment', action=argparse.BooleanOptionalAction)
    parser.add_argument('--seed', type=int, default=0,
                        help='random seed (default: 0)')
    parser.add_argument('--log-dir', type=str,
                        default='', help='log dir')
    parser.add_argument('--model-path', type=str,
                        default='', help='model load/store path')
    parser.add_argument('--car', type=str, default='husky',
                        help='car model to use')
    parser.add_argument('--car-scale', type=float, default=1.1,
                        help='scaling of the car model')
    parser.add_argument('--real', type=bool, default=False,
                        help='Real world or not', action=argparse.BooleanOptionalAction)
    parser.add_argument('--mode', type=str, default='1',
                        choices=['1', '2', '3', '4', '5', '6'], help='mode')
    parser.add_argument('--eval-episodes', type=int,
                        default=int(10), help='total episodes to eval')
    f(parser)
    return parser


def grab_args(parser):
    args = parser.parse_args()
    time = datetime.strftime(datetime.now(), '%m%d_%H%M')
    default_log_dir = LOG_DIR/f'{args.model}_{args.mode}_{time}'
    if not args.log_dir:
        args.log_dir = str(default_log_dir)
    return args
