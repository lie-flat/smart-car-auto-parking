from stable_baselines3 import DQN, PPO
from functools import partial


def get_model_class_by_name(name):
    match name:
        case 'dqn':
            return DQN
        case 'ppo':
            return PPO
        case _:
            raise ValueError(f"Invalid model {name}!")

def get_model_ctor_by_name(name):
    match name:
        case 'dqn':
            return partial(DQN,  exploration_fraction=0.3,
                           exploration_initial_eps=1.0,
                           exploration_final_eps=0.05,)
        case 'ppo':
            return PPO
        case _:
            raise ValueError(f"Invalid model {name}!")


def init_model_by_name(name, **kwargs):
    model_class = get_model_ctor_by_name(name)
    return model_class('MlpPolicy', **kwargs)
