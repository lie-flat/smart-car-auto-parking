# Adapted from https://github.com/RyanLiu112/RL_parking ,
# which originates from https://github.com/Robotics-Club-IIT-BHU/gym-carpark

import gym
import numpy as np
from gym import spaces
from abc import abstractmethod

from ..config.rl import TARGET_X, TARGET_Y
from .analytics import AnalyticsCollector


class ParkingLotEnvBase(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, max_steps=500, init_x=-1.5, init_y=1.45, init_theta=np.pi, enable_collector=False):
        """
        初始化环境
        """
        super().__init__()
        self.collector = AnalyticsCollector() if enable_collector else None

        self.loaded = False
        self.done = False
        self.goal = None
        self.desired_goal = None
        self.init_position = [init_x, init_y, 0.2]
        self.init_orientation = [0, 0, init_theta]

        # 定义状态空间
        obs_low = np.array([-4, -4, -1, -1, -1, -1], dtype=np.float32)
        obs_high = np.array([4, 4, 1, 1, 1, 1], dtype=np.float32)
        self.observation_space = spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float32)

        # 定义动作空间
        self.action_space = spaces.Discrete(4)  # 4种动作：前进、后退、左转、右转
        self.reward_weights = np.array([1, 0.3, 0, 0, 0.1, 0.1])
        self.goal = np.array([TARGET_X, TARGET_Y])
        self.target_orientation = 2 * np.pi / 3

        self.desired_goal = np.array([self.goal[0], self.goal[1], 0.0, 0.0, np.cos(
            self.target_orientation), np.sin(self.target_orientation)])

        self.action_steps = 5
        self.step_cnt = 0
        self.max_steps = max_steps
        self.cummulative_reward = 0

    @abstractmethod
    def _load_env(self):
        """
        Load the environment
        """
        raise NotImplementedError

    def reset(self):
        """
        重置环境
        """
        if not self.loaded:
            self._load_env()
            self.loaded = True
        self.step_cnt = 0
        self.cummulative_reward = 0
        return None

    def distance_function(self, pos):
        """
        计算小车与目标点的距离（2-范数）

        :param pos: 小车当前坐标 [x, y, z]
        :return: 小车与目标点的距离
        """

        return np.sqrt(pow(pos[0] - self.goal[0], 2) + pow(pos[1] - self.goal[1], 2))

    def compute_reward(self, obs):
        """
        计算当前步的奖励
        """
        return -np.sqrt(np.dot(np.abs(obs - self.desired_goal), self.reward_weights))

    def seed(self, seed=None):
        """
        设置环境种子

        :param seed: 种子
        :return: [seed]
        """
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
