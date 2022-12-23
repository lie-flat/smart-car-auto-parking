# Adapted from https://github.com/RyanLiu112/RL_parking ,
# which originates from https://github.com/Robotics-Club-IIT-BHU/gym-carpark

from math import pi
import pybullet as p
import numpy as np
from functools import partial


from ..config import ENVIRONMENT_RESOURCES_DIR
from ..controller import control, act


class Car:
    def __init__(self, client, base_position=None, base_orientation_euler=None,
                 max_velocity=6, max_force=100, car_type='husky', action_steps=5, scale=1.1, real_car_ip=None):
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
        self.type = car_type
        match car_type:
            case 'husky':
                urdf_path = f"{car_type}/{car_type}.urdf"
                self.apply_action = self.husky_control
            case 'racecar':
                urdf_path = f"{car_type}/{car_type}.urdf"
                self.apply_action = self.racecar_control
                self.steering_joints = [4, 6]
                self.drive_joints = [2, 3, 7, 5]
            case car:
                urdf_path = str(ENVIRONMENT_RESOURCES_DIR / f"{car}.urdf")
                self.apply_action = self.car_control
        self.id = p.loadURDF(fileName=urdf_path, basePosition=base_position,
                             baseOrientation=self.base_orientation, globalScaling=scale)

        self.max_velocity = max_velocity
        self.max_force = max_force
        self.action_steps = action_steps
        self.ip = real_car_ip
        self.real_act = partial(act, self.ip)
        self.real_ctrl = partial(control, self.ip)

    def real_car_control(self, action):
        duration = 5 * int(self.action_steps * 1000/240)
        turn_duration = 4 * duration
        turn_speed = 60
        speed = 40
        print(duration)
        match action:
            case 0:
                self.real_act(a=speed, duration=duration)
            case 1:
                self.real_act(b=speed, duration=duration)
            case 2:
                self.real_act(servo=12.5, b=turn_speed, duration=turn_duration)
            case 3:
                self.real_act(servo=2.5, b=turn_speed, duration=turn_duration)
            case 4:
                self.real_act(duration=duration)
            case _:
                raise ValueError("Invalid action!")

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

    def racecar_control(self, action):
        """
        自定义小车控制
        """
        steering_angle = pi/6
        force = [20] * 4
        steering_angle = pi/6
        velocity = 2
        self.action_steps = 3
        match action:
            case 0:
                p.setJointMotorControlArray(bodyUniqueId=self.id, jointIndices=self.drive_joints,
                                            controlMode=p.VELOCITY_CONTROL, targetVelocities=[velocity]*4, forces=force, physicsClientId=self.client)
                for _ in range(self.action_steps):
                    p.stepSimulation()
            case 1:
                p.setJointMotorControlArray(bodyUniqueId=self.id, jointIndices=self.drive_joints,
                                            controlMode=p.VELOCITY_CONTROL, targetVelocities=[-velocity]*4, forces=force, physicsClientId=self.client)
                for _ in range(self.action_steps):
                    p.stepSimulation()
            case 2:
                p.setJointMotorControlArray(self.id, self.steering_joints,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPositions=[steering_angle]*2,
                                            physicsClientId=self.client)
                for _ in range(self.action_steps):
                    p.stepSimulation()
            case 3:
                p.setJointMotorControlArray(self.id, self.steering_joints,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPositions=[-steering_angle]*2,
                                            physicsClientId=self.client)
                for _ in range(self.action_steps):
                    p.stepSimulation()
            case 4:
                p.setJointMotorControlArray(bodyUniqueId=self.id, jointIndices=self.drive_joints,
                                            controlMode=p.VELOCITY_CONTROL, targetVelocities=[0]*4, forces=force, physicsClientId=self.client)
                p.setJointMotorControlArray(self.id, self.steering_joints,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPositions=[0]*2,
                                            physicsClientId=self.client)
                for _ in range(self.action_steps):
                    p.stepSimulation()
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
        if self.ip is not None:
            self.real_car_control(action)
            print(action, end='')

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
        if self.type == 'racecar':
            p.setJointMotorControlArray(self.id, self.steering_joints,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=[0]*2,
                                        physicsClientId=self.client)
            p.setJointMotorControlArray(bodyUniqueId=self.id, jointIndices=self.drive_joints,
                                        controlMode=p.VELOCITY_CONTROL, targetVelocities=[0]*4, forces=[0, 0, 0, 0], physicsClientId=self.client)
