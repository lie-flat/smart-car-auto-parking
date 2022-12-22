import gym
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
from colorama import Fore, Style
from sys import stderr
from .models import init_model_by_name, get_model_by_name


def train(args):
    env = gym.make(args.env, render=args.render, mode=args.mode)
    checkpoint_callback = CheckpointCallback(
        save_freq=args.save_freq, save_path=args.ckpt_path, name_prefix=args.model)
    model = init_model_by_name(args.model, env=env, verbose=1, seed=args.seed)
    logger = configure(args.log_dir, ["tensorboard"])
    model.set_logger(logger)
    model.learn(total_timesteps=args.total_steps,
                callback=checkpoint_callback)
    model.save(args.model_path)
    env.close()


def evaluate(args):
    env = Monitor(gym.make(args.env, render=args.render, mode=args.mode))
    model_class = get_model_by_name(args.model)
    model = model_class.load(args.model_path, env)
    mean, std = evaluate_policy(
        model, env, n_eval_episodes=10, render=args.render)
    print(f"{Fore.YELLOW}Mean reward: {mean}, Std: {std}{Style.RESET_ALL}", file=stderr)