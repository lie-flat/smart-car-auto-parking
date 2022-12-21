import gym
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure


from .models import init_model_by_name


def train(args):
    env = gym.make(args.env, render=args.render, mode=args.mode)
    checkpoint_callback = CheckpointCallback(
        save_freq=args.save_freq, save_path=args.ckpt_path, name_prefix=args.model)
    model = init_model_by_name(args.model, env=env, verbose=1, seed=args.seed)
    logger = configure(args.log_dir, ["tensorboard"])
    model.set_logger(logger)
    model.learn(total_timesteps=args.total_steps,
                callback=checkpoint_callback)
    model.save(args.save_path)
    env.close()

def eval(args):
    