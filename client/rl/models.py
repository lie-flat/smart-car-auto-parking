from stable_baselines3 import DQN, PPO


def get_model_by_name(name):
    match name:
        case 'dqn':
            return DQN
        case 'ppo':
            return PPO
        case _:
            raise ValueError(f"Invalid model {name}!")


def init_model_by_name(name, **kwargs):
    model_class = get_model_by_name(name)
    return model_class('MlpPolicy', **kwargs)
