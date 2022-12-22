import argparse
from .cmd_parser import build_parser, grab_args
from .impl import train, evaluate


def build_train_parser():
    def builder(parser):
        parser.add_argument('--total-steps', type=int,
                            default=int(2e5), help='total steps to run')
        parser.add_argument('--save-freq', type=int, default=int(5e4),
                            help='checkpoint save frequency')
        parser.add_argument('--ckpt-path', type=str,
                            default='', help='checkpoint path')
        parser.add_argument('--resume-from', type=str,
                            default='', help='model/run to resume from')
    return build_parser(builder)


if __name__ == '__main__':
    parser = build_train_parser()
    args = grab_args(parser)
    train(args)
    if args.eval_episodes > 0:
        evaluate(args)
