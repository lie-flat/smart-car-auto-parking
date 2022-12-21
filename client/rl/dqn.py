# Adapted from https://github.com/RyanLiu112/RL_parking

from os import path
import gym
from stable_baselines3 import DQN
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure

from ..config.rl import LOG_DIR
from .cmd_parser import grab_args
from .funcs import train

LOG_DIR = str(LOG_DIR)

if __name__ == '__main__':
    args = grab_args()
    train(args)
    # env = gym.make(args.env, render=args.render, mode=args.mode)
    # env.reset()
    # model = DQN('MlpPolicy', env, verbose=1, seed=args.seed)
    # logger = configure(args.log_dir, ["csv", "tensorboard"])
    # model.set_logger(logger)
    # checkpoint_callback = CheckpointCallback(
    #     save_freq=args.save_freq, save_path=args.ckpt_path, name_prefix='dqn_agent')
    # model.learn(total_timesteps=args.total_steps,
    #             callback=checkpoint_callback)
    # model.save(args.save_path)
    # env.close()

    # Evaluation

    # env = gym.make(args.env, render=True, mode=args.mode)
    # model = DQN.load(args.save_path, env=env)

    # episode_return = 0
    # for i in range(1000):
    #     action, _ = model.predict(obs, deterministic=True)
    #     obs, reward, done, info = env.step(action)
    #     episode_return += reward
    #     if done:
    #         break

    # env.close()
    # print(f'episode return: {episode_return}')
