from .cmd_parser import build_parser, grab_args
from .impl import evaluate


if __name__ == '__main__':
    parser = build_parser()
    args = grab_args(parser)
    evaluate(args)
