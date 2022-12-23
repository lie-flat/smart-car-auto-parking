# Adapted from https://github.com/RyanLiu112/RL_parking ,
# which originates from https://github.com/Robotics-Club-IIT-BHU/gym-carpark

import pybullet as p
import numpy as np


from ..config import ENVIRONMENT_RESOURCES_DIR


class Car:
    def __init__(self, client, base_position=None, base_orientation_euler=None,
                 max_velocity=6, max_force=100, car_type='husky', action_steps=5, scale=1.1):
        """
        初始化小车

        :param client: pybullet client
        :param base_position: 小车初始位置
        :param base_orientation_euler: 小车初始方向
        :param max_velocity: 最大速度
        :param max_force: 最大力
        :param car_type: 小车类型
        :param action_steps: 动作步数
        """
        self.base_position = base_position or [0, 0, 0.2]
        self.base_orientation = p.getQuaternionFromEuler(
            base_orientation_euler or [0, 0, np.pi / 2])

        self.client = client
        match car_type:
            case 'husky':
                urdf_path = f"{car_type}/{car_type}.urdf"
                self.apply_action = self.husky_control
            case car:
                urdf_path = str(ENVIRONMENT_RESOURCES_DIR / f"{car}.urdf")
                self.apply_action = self.car_control
        self.id = p.loadURDF(fileName=urdf_path, basePosition=base_position,
                             baseOrientation=self.base_orientation, globalScaling=scale)

        self.steering_joints = [0, 2]
        self.drive_joints = [1, 3, 4, 5]

        self.max_velocity = max_velocity
        self.max_force = max_force
        self.action_steps = action_steps

    def car_control(self, action):
        """
        自定义小车控制
        """
        match action:
            case 0:
                pass
            case 1:
                pass
            case 2:
                pass
            case 3:
                pass
            case _:
                raise ValueError("Invalid action!")

    def husky_control(self, action):
        """
        Husky 小车控制
        """

        velocity = self.max_velocity
        force = self.max_force
        match action:
            case 0:  # 前进
                for _ in range(self.action_steps):
                    for joint in range(2, 6):
                        p.setJointMotorControl2(
                            self.id, joint, p.VELOCITY_CONTROL, targetVelocity=velocity, force=force)
                    p.stepSimulation()
            case 1:  # 后退
                for _ in range(self.action_steps):
                    for joint in range(2, 6):
                        p.setJointMotorControl2(
                            self.id, joint, p.VELOCITY_CONTROL, targetVelocity=-velocity, force=force)
                    p.stepSimulation()
            case 2:  # 左转
                targetVel = 3
                for _ in range(self.action_steps * 4):
                    # for joint in range(2, 6):
                    for joint in range(1, 3):
                        p.setJointMotorControl2(self.id, 2 * joint + 1, p.VELOCITY_CONTROL,
                                                targetVelocity=targetVel, force=force)
                    for joint in range(1, 3):
                        p.setJointMotorControl2(self.id, 2 * joint, p.VELOCITY_CONTROL, targetVelocity=-targetVel,
                                                force=force)
                    p.stepSimulation()
            case 3:  # 右转
                targetVel = 3
                for _ in range(self.action_steps * 4):
                    # for joint in range(2, 6):
                    for joint in range(1, 3):
                        p.setJointMotorControl2(self.id, 2 * joint, p.VELOCITY_CONTROL, targetVelocity=targetVel,
                                                force=force)
                    for joint in range(1, 3):
                        p.setJointMotorControl2(self.id, 2 * joint + 1, p.VELOCITY_CONTROL,
                                                targetVelocity=-targetVel, force=force)
                    p.stepSimulation()
            case 4:  # 停止
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(self.id, joint, p.VELOCITY_CONTROL, targetVelocity=targetVel,
                                            force=force)
                p.stepSimulation()
            case _:
                raise ValueError("Invalid action!")

    def get_observation(self):
        """
        获取小车当前状态

        :return: observation, vector
        """

        position, angle = p.getBasePositionAndOrientation(self.id)  # 获取小车位姿
        angle = p.getEulerFromQuaternion(angle)
        velocity = p.getBaseVelocity(self.id)[0]

        position = [position[0], position[1]]
        velocity = [velocity[0], velocity[1]]
        orientation = [np.cos(angle[2]), np.sin(angle[2])]
        vector = angle[2]

        observation = np.array(position + velocity + orientation)  # 拼接坐标、速度、角度

        return observation, vector

    def reset(self):
        p.resetBasePositionAndOrientation(
            self.id, self.base_position, self.base_orientation)
