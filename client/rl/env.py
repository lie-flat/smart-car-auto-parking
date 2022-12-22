# Adapted from https://github.com/RyanLiu112/RL_parking ,
# which originates from https://github.com/Robotics-Club-IIT-BHU/gym-carpark

import gym
import pybullet as p
import pybullet_data
import numpy as np
import time
from gym import spaces

from .car import Car
from ..config import ENVIRONMENT_RESOURCES_DIR
from ..config.rl import TARGET_AREA_BOTTOM_LEFT, TARGET_AREA_BOTTOM_RIGHT, TARGET_AREA_TOP_LEFT, TARGET_AREA_TOP_RIGHT, TARGET_X, TARGET_Y


class ParkingLotEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, render=False, car_type='husky', mode='1', manual=False, render_video=False):
        """
        初始化环境

        :param render: 是否渲染GUI界面
        :param car_type: 小车类型（husky）
        :param mode: 任务类型
        :param manual: 是否手动操作
        :param render_video: 是否渲染视频
        """
        self.loaded = False
        self.car_type = car_type
        self.manual = manual
        self.mode = mode
        assert self.mode in ['1', '2', '3', '4', '5', '6']

        self.car = None
        self.done = False
        self.goal = None
        self.desired_goal = None

        self.ground = None
        # 定义状态空间
        obs_low = np.array([0, 0, -1, -1, -1, -1], dtype=np.float32)
        obs_high = np.array([20, 20, 1, 1, 1, 1], dtype=np.float32)
        self.observation_space = spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float32)

        # 定义动作空间
        self.action_space = spaces.Discrete(4)  # 4种动作：前进、后退、左转、右转

        # self.reward_weights = np.array([1, 0.3, 0, 0, 0.02, 0.02])
        if self.mode == '4':
            self.reward_weights = np.array([0.4, 0.9, 0, 0, 0.1, 0.1])
        elif self.mode == '5':
            self.reward_weights = np.array([0.65, 0.65, 0, 0, 0.1, 0.1])
        else:
            self.reward_weights = np.array([1, 0.3, 0, 0, 0.1, 0.1])
        self.target_orientation = None
        self.start_orientation = None

        if self.mode in ['1', '2', '3', '5', '6']:
            self.action_steps = 5
        else:
            self.action_steps = 3
        self.step_cnt = 0
        self.step_threshold = 500

        if render:
            self.client = p.connect(p.GUI)
            time.sleep(1. / 240.)
        else:
            self.client = p.connect(p.DIRECT)
            time.sleep(1. / 240.)
        if render and render_video:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

    def render(self, mode='human'):
        """
        渲染当前画面

        :param mode: 渲染模式
        """

        p.stepSimulation(self.client)
        time.sleep(1. / 240.)

    def _load_env(self):
        """
        Load 3d objects
        """
        # 加载地面
        p.setGravity(0, 0, -10)
        self.ground = p.loadURDF(
            str(ENVIRONMENT_RESOURCES_DIR/"ground.urdf"), basePosition=[0, 0, 0.005], useFixedBase=10)
        thickness = 2.5
        color = [0.98, 0.98, 0.98]
        p.addUserDebugLine(TARGET_AREA_TOP_LEFT,
                           TARGET_AREA_TOP_RIGHT, color, thickness)
        p.addUserDebugLine(TARGET_AREA_TOP_LEFT,
                           TARGET_AREA_BOTTOM_LEFT, color, thickness)
        p.addUserDebugLine(TARGET_AREA_BOTTOM_RIGHT,
                           TARGET_AREA_BOTTOM_LEFT, color, thickness)
        p.addUserDebugLine(TARGET_AREA_TOP_RIGHT,
                           TARGET_AREA_BOTTOM_RIGHT, color, thickness)
        # self.basePosition = [-1.5, 1.4, 0.2]
        # self.basePosition = [-0.2, 1.4, 0.2] # 直线入库
        self.basePosition = [-1.5, 1.45, 0.2] # 斜方入库1
        # self.basePosition = [TARGET_X, TARGET_Y, 0.2]
        self.goal = np.array([TARGET_X, TARGET_Y])
        # self.start_orientation = [0, 0, np.pi * 2 / 2]
        self.start_orientation = [0, 0, np.pi]
        self.target_orientation = 2.070143
        self.desired_goal = np.array([self.goal[0], self.goal[1], 0.0, 0.0, np.cos(
            self.target_orientation), np.sin(self.target_orientation)])
        self.car = Car(self.client, base_position=self.basePosition, base_orientation_euler=self.start_orientation,
                       car_type=self.car_type, action_steps=self.action_steps)

    def reset(self):
        """
        重置环境

        """
        if not self.loaded:
            self._load_env()
            self.loaded = True

        # 重置小车
        # p.removeBody(self.car.id)
        # self.car = Car(self.client, base_position=self.basePosition, base_orientation_euler=self.start_orientation,
        #    car_type=self.car_type, action_steps=self.action_steps)
        # p.resetBasePositionAndOrientation(self.car.id)
        self.car.reset()
        # 获取当前observation
        car_ob, self.vector = self.car.get_observation()
        observation = np.array(list(car_ob))

        self.step_cnt = 0

        return observation

    def distance_function(self, pos):
        """
        计算小车与目标点的距离（2-范数）

        :param pos: 小车当前坐标 [x, y, z]
        :return: 小车与目标点的距离
        """

        return np.sqrt(pow(pos[0] - self.goal[0], 2) + pow(pos[1] - self.goal[1], 2))

    def compute_reward(self, achieved_goal, desired_goal, info):
        """
        计算当前步的奖励

        :param achieved_goal: 小车当前位置 [x, y, z]
        :param desired_goal: 目标点 [x, y, z]
        :param info: 信息
        :return: 奖励
        """

        p_norm = 0.5
        reward = -np.power(np.dot(np.abs(achieved_goal - desired_goal),
                                  np.array(self.reward_weights)), p_norm)

        return reward

    def judge_collision(self):
        """
        判断小车与墙壁、停放着的小车是否碰撞

        :return: 是否碰撞
        """

        done = False

        return done

    def step(self, action):
        """
        环境步进

        :param action: 小车动作
        :return: observation, reward, done, info
        """

        self.car.apply_action(action)  # 小车执行动作
        p.stepSimulation()
        car_ob, self.vector = self.car.get_observation()  # 获取小车状态

        position = np.array(car_ob[:2])
        distance = self.distance_function(position)
        reward = self.compute_reward(car_ob, self.desired_goal, None)

        if self.manual:
            print(
                f'dis: {distance}, reward: {reward}, center: {self.goal}, pos: {car_ob}')

        self.done = False
        self.success = False

        if distance < 0.2:
            self.success = True
            self.done = True

        self.step_cnt += 1
        if self.step_cnt > self.step_threshold:  # 限制episode长度为step_threshold
            self.done = True
        if car_ob[2] < -2:  # 小车掉出环境
            # print('done! out')
            reward = -500
            self.done = True
        if self.judge_collision():  # 碰撞
            # print('done! collision')
            reward = -500
            self.done = True
        if self.done:
            self.step_cnt = 0

        observation = np.array(list(car_ob))
        info = {'is_success': self.success}

        return observation, reward, self.done, info

    def seed(self, seed=None):
        """
        设置环境种子

        :param seed: 种子
        :return: [seed]
        """
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def close(self):
        """
        关闭环境
        """
        p.disconnect(self.client)
