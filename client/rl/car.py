# Adapted from https://github.com/RyanLiu112/RL_parking ,
# which originates from https://github.com/Robotics-Club-IIT-BHU/gym-carpark

import pybullet as p
import numpy as np


class Car:
    def __init__(self, client, basePosition=None, baseOrientationEuler=None,
                 max_velocity=6, max_force=100, carType='husky', action_steps=None):
        """
        初始化小车

        :param client: pybullet client
        :param basePosition: 小车初始位置
        :param baseOrientationEuler: 小车初始方向
        :param max_velocity: 最大速度
        :param max_force: 最大力
        :param carType: 小车类型
        :param action_steps: 动作步数
        """
        self.basePosition = basePosition or [0, 0, 0.2]
        self.baseOrientationEuler = baseOrientationEuler or [0, 0, np.pi / 2]

        self.client = client
        urdfname = carType + '/' + carType + '.urdf'
        self.car = p.loadURDF(fileName=urdfname, basePosition=basePosition,
                              baseOrientation=p.getQuaternionFromEuler(baseOrientationEuler))

        self.steering_joints = [0, 2]
        self.drive_joints = [1, 3, 4, 5]

        self.max_velocity = max_velocity
        self.max_force = max_force
        self.action_steps = action_steps

    def apply_action(self, action):
        """
        小车执行动作

        :param action: 动作
        """

        velocity = self.max_velocity
        force = self.max_force

        if action == 0:  # 前进
            for _ in range(self.action_steps):
                for joint in range(2, 6):
                    p.setJointMotorControl2(
                        self.car, joint, p.VELOCITY_CONTROL, targetVelocity=velocity, force=force)
                p.stepSimulation()
        elif action == 1:  # 后退
            for _ in range(self.action_steps):
                for joint in range(2, 6):
                    p.setJointMotorControl2(
                        self.car, joint, p.VELOCITY_CONTROL, targetVelocity=-velocity, force=force)
                p.stepSimulation()
        elif action == 2:  # 左转
            targetVel = 3
            for _ in range(self.action_steps):
                for joint in range(2, 6):
                    for joint in range(1, 3):
                        p.setJointMotorControl2(self.car, 2 * joint + 1, p.VELOCITY_CONTROL,
                                                targetVelocity=targetVel, force=force)
                    for joint in range(1, 3):
                        p.setJointMotorControl2(self.car, 2 * joint, p.VELOCITY_CONTROL, targetVelocity=-targetVel,
                                                force=force)
                    p.stepSimulation()
        elif action == 3:  # 右转
            targetVel = 3
            for _ in range(self.action_steps):
                for joint in range(2, 6):
                    for joint in range(1, 3):
                        p.setJointMotorControl2(self.car, 2 * joint, p.VELOCITY_CONTROL, targetVelocity=targetVel,
                                                force=force)
                    for joint in range(1, 3):
                        p.setJointMotorControl2(self.car, 2 * joint + 1, p.VELOCITY_CONTROL,
                                                targetVelocity=-targetVel, force=force)
                    p.stepSimulation()
        elif action == 4:  # 停止
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(self.car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel,
                                        force=force)
            p.stepSimulation()
        else:
            raise ValueError

    def get_observation(self):
        """
        获取小车当前状态

        :return: observation, vector
        """

        position, angle = p.getBasePositionAndOrientation(self.car)  # 获取小车位姿
        angle = p.getEulerFromQuaternion(angle)
        velocity = p.getBaseVelocity(self.car)[0]

        position = [position[0], position[1]]
        velocity = [velocity[0], velocity[1]]
        orientation = [np.cos(angle[2]), np.sin(angle[2])]
        vector = angle[2]

        observation = np.array(position + velocity + orientation)  # 拼接坐标、速度、角度

        return observation, vector
