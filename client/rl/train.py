from .cmd_parser import build_train_parser, grab_args
from .impl import train, evaluate


if __name__ == '__main__':
    parser = build_train_parser()
    args = grab_args(parser)
    train(args)
    if args.eval:
        evaluate(args)
