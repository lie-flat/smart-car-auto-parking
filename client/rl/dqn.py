# Adapted from https://github.com/RyanLiu112/RL_parking

import argparse
import datetime
import os

import gym
from stable_baselines3 import DQN
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from ..config.rl import LOG_DIR

LOG_DIR = str(LOG_DIR)

parser = argparse.ArgumentParser()
parser.add_argument('--env', type=str, default="ParkingLot-v0",
                    help='name of the environment to run')
parser.add_argument('--render', type=bool, default=False,
                    help='render the environment')
parser.add_argument('--seed', type=int, default=0,
                    help='random seed (default: 0)')
parser.add_argument('--total_timesteps', type=int,
                    default=int(2e5), help='total timesteps to run')
parser.add_argument('--save_freq', type=int, default=int(5e4),
                    help='checkpoint save frequency')
parser.add_argument('--ckpt_path', type=str,
                    default='', help='checkpoint path')
parser.add_argument('--mode', type=str, default='1',
                    choices=['1', '2', '3', '4', '5', '6'], help='mode')

args = parser.parse_args()

time = datetime.datetime.strftime(datetime.datetime.now(), '%m%d_%H%M')
current_run_log_dir = os.path.join(LOG_DIR, f'DQN_{args.mode}_{time}')


if not args.ckpt_path:
    args.ckpt_path = os.path.join(current_run_log_dir, f'dqn_agent')

env = gym.make(args.env, render=args.render, mode=args.mode)
env.reset()

model = DQN('MlpPolicy', env, verbose=1, seed=args.seed)
logger = configure(current_run_log_dir, ["csv", "tensorboard"])
model.set_logger(logger)
checkpoint_callback = CheckpointCallback(
    save_freq=args.save_freq, save_path=current_run_log_dir, name_prefix='dqn_agent')
model.learn(total_timesteps=args.total_timesteps, callback=checkpoint_callback)
model.save(args.ckpt_path)

del model


# Evaluation
env = gym.make(args.env, render=True, mode=args.mode)
obs = env.reset()
model = DQN.load(args.ckpt_path, env=env)

episode_return = 0
for i in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    episode_return += reward
    if done:
        for j in range(10000000):
            reward += 0.0001
        break

env.close()
print(f'episode return: {episode_return}')
