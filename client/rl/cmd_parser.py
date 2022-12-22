import argparse
from datetime import datetime
from os import path, makedirs


from ..config.rl import LOG_DIR


def build_parser(f):
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
    parser.add_argument('--mode', type=str, default='1',
                        choices=['1', '2', '3', '4', '5', '6'], help='mode')
    f(parser)
    return parser


def build_train_parser():
    def builder(parser):
        parser.add_argument('--total-steps', type=int,
                            default=int(2e5), help='total steps to run')
        parser.add_argument('--save-freq', type=int, default=int(5e4),
                            help='checkpoint save frequency')
        parser.add_argument('--ckpt-path', type=str,
                            default='', help='checkpoint path')
        parser.add_argument('--eval', type=bool, default=True,
                            help='evaluate after training', action=argparse.BooleanOptionalAction)
    return build_parser(builder)


def grab_args(parser):
    args = parser.parse_args()
    time = datetime.strftime(datetime.now(), '%m%d_%H%M')
    default_log_dir = LOG_DIR/f'{args.model}_{args.mode}_{time}'
    if not args.log_dir:
        args.log_dir = str(default_log_dir)
    if not args.ckpt_path:
        args.ckpt_path = str(default_log_dir/'ckpt')
    makedirs(args.log_dir, exist_ok=True)
    makedirs(args.ckpt_path, exist_ok=True)
    args.model_path = path.join(args.log_dir, "final")
    return args
