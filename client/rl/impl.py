import gym
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
from colorama import Fore, Style
from sys import stderr
from pathlib import Path
from .models import init_model_by_name, get_model_class_by_name


def make_env(args):
    return gym.make(args.env, render=args.render, car_type=args.car,
                    init_x=args.init_x, init_y=args.init_y, init_theta=args.init_theta,
                    car_scaling=args.car_scale, real=args.real, presentation_mode=args.presentation)


def train(args):
    env = make_env(args)
    checkpoint_callback = CheckpointCallback(
        save_freq=args.save_freq, save_path=args.ckpt_path, name_prefix=args.model)
    if args.resume_from:
        model_class = get_model_class_by_name(args.model)
        path = Path(args.resume_from)
        if path.is_file():
            model_path = args.resume_from
        else:
            model = str(path/'final.zip')
        model = model_class.load(model_path, env=env)
    else:
        model = init_model_by_name(
            args.model, env=env, verbose=1, seed=args.seed)
    logger = configure(args.log_dir, ["tensorboard"])
    model.set_logger(logger)
    env.reset()
    model.learn(total_timesteps=args.total_steps,
                callback=checkpoint_callback)
    model.save(args.model_path)
    env.close()


def evaluate(args):
    env = Monitor(make_env(args))
    model_class = get_model_class_by_name(args.model)

    path = Path(args.model_path)
    model_path = str(path).removesuffix(
        ".zip") if path.is_file() else str(path/'final')
    model = model_class.load(model_path, env)
    env.reset()
    mean, std = evaluate_policy(
        model, env, n_eval_episodes=args.eval_episodes, render=args.render)
    print(f"{Fore.YELLOW}Mean reward: {mean}, Std: {std}{Style.RESET_ALL}", file=stderr)
