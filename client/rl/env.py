# Adapted from https://github.com/RyanLiu112/RL_parking ,
# which originates from https://github.com/Robotics-Club-IIT-BHU/gym-carpark

import pybullet as p
import pybullet_data
import numpy as np
import time
from math import pi

from .car import Car
from .base import ParkingLotEnvBase
from ..config import ENVIRONMENT_RESOURCES_DIR
from ..config.rl import TARGET_AREA_BOTTOM_LEFT, TARGET_AREA_BOTTOM_RIGHT, TARGET_AREA_TOP_LEFT, TARGET_AREA_TOP_RIGHT, TARGET_X, TARGET_Y
from ..controller import connect_to_board


class ParkingLotEnv(ParkingLotEnvBase):
    metadata = {'render.modes': ['human']}

    def __init__(self, render=False, car_type='husky', car_scaling=1.1,
                 max_steps=500, real=False, view=False, init_x=-1.5, init_y=1.45, init_theta=np.pi):
        """
        初始化环境
        """
        super().__init__(max_steps=max_steps, init_x=init_x,
                         init_y=init_y, init_theta=init_theta)
        self.car_type = car_type
        self.car_scaling = car_scaling
        self.view = view
        self.car = None

        self.walls = []
        self.ground = None

        if render:
            self.client = p.connect(
                p.GUI, options='--width=640 --height=480' if view else '')
            # Disable default controls
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=3.2,
                cameraYaw=0,
                cameraPitch=-75,
                cameraTargetPosition=[0, 0, 0]
            )
        else:
            self.client = p.connect(p.DIRECT)
        time.sleep(1. / 240.)
        if real:
            devices = connect_to_board()
            self.real_car_ip = devices['board']
        else:
            self.real_car_ip = None
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

    def render(self, _mode):
        """
        渲染当前画面
        """
        p.stepSimulation(self.client)
        time.sleep(1. / 240.)

    def _load_env(self):
        """
        Load 3d objects
        """
        # 加载地面
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
        # Load walls
        self.walls.append(p.loadURDF(str(ENVIRONMENT_RESOURCES_DIR/"wall.urdf"),
                                     basePosition=[-0.3, 0.3, 0], baseOrientation=p.getQuaternionFromEuler([0, 0, -pi/3]), useFixedBase=10))
        self.walls.append(p.loadURDF(str(ENVIRONMENT_RESOURCES_DIR/"wall.urdf"),
                                     basePosition=[0.95, 0.3, 0], baseOrientation=p.getQuaternionFromEuler([0, 0, -pi/3]), useFixedBase=10))
        # self.basePosition = [-1.5, 1.4, 0.2]
        # self.basePosition = [-0.2, 1.4, 0.2] # 直线入库
        # self.basePosition = [-1.5, 1.45, 0.2]  # 斜方入库1
        # self.basePosition = [TARGET_X, TARGET_Y, 0.2]

        self.car = Car(self.client, base_position=self.init_position, base_orientation_euler=self.init_orientation,
                       car_type=self.car_type, scale=self.car_scaling, action_steps=self.action_steps, real_car_ip=self.real_car_ip)

    def reset(self):
        """
        重置环境
        """
        super().reset()

        # 重置小车
        self.car.reset()
        # 获取当前observation
        car_ob, self.vector = self.car.get_observation()
        observation = np.array(list(car_ob))

        self.step_cnt = 0

        return observation

    def judge_collision(self):
        """
        判断小车与墙壁是否碰撞
        """
        for wall in self.walls:
            if len(p.getContactPoints(self.car.id, wall)) > 0:
                return True
        return False

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
        reward = self.compute_reward(car_ob)

        self.done = False
        self.success = False

        if distance < 0.15:
            self.success = True
            self.done = True

        self.step_cnt += 1
        if self.step_cnt > self.max_steps:  # 限制episode长度为step_threshold
            print("MAxStEp")
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

    def close(self):
        """
        关闭环境
        """
        p.disconnect(self.client)
