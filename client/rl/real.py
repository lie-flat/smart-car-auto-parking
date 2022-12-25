import numpy as np
from functools import partial
from filelock import FileLock
from multiprocessing.shared_memory import SharedMemory
import gym

from .cmd_parser import build_parser, grab_args
from .impl import evaluate
from .base import ParkingLotEnvBase
from ..config import BASE_DIR
from ..controller import connect_to_board, act, control
from ..config.rl import REAL_CAR_SPEED, REAL_CAR_TURN_SPEED, FEEDBACK_SHM_NAME, FEEDBACK_FILELOCK_PATH, FEEDBACK_DTYPE, FEEDBACK_SIZE


class RealParkingLotEnv(ParkingLotEnvBase):
    def __init__(self, max_steps=500):
        """
        初始化环境
        """
        super().__init__(max_steps=max_steps, enable_collector=True)
        devices = connect_to_board()
        ip = devices['board']
        self.lock = FileLock(BASE_DIR/"feedback.shm.lock")
        self.shm = SharedMemory(name=FEEDBACK_SHM_NAME)
        self.observation_buffer = np.ndarray(FEEDBACK_SIZE, dtype=FEEDBACK_DTYPE,
                                             buffer=self.shm.buf)
        self.act = partial(act, ip)
        self.ctrl = partial(control, ip)
        self.observation = np.zeros(6, dtype=FEEDBACK_DTYPE)

    def _load_env(self):
        input("Press <Enter> when you are ready to start!")

    def reset(self):
        super().reset()
        return self.collect_observation()

    def collect_observation(self):
        with self.lock:
            self.observation[:] = self.observation_buffer
        return self.observation

    def execute_action(self, action):
        duration = 5 * int(self.action_steps * 1000/240)
        turn_duration = 4 * duration
        print(duration)
        match action:
            case 0:
                self.act(a=REAL_CAR_SPEED, duration=duration)
            case 1:
                self.act(b=REAL_CAR_SPEED, duration=duration)
            case 2:
                self.act(servo=12.5, b=REAL_CAR_TURN_SPEED,
                         duration=turn_duration)
            case 3:
                self.act(servo=2.5, b=REAL_CAR_TURN_SPEED,
                         duration=turn_duration)
            case 4:
                self.act(duration=duration)
            case _:
                raise ValueError("Invalid action!")

    def step(self, action):
        self.execute_action(action)
        self.collect_observation()

        observation = self.collect_observation()

        position = observation[:2]
        distance = self.distance_function(position)
        reward = self.compute_reward(observation)

        self.done = False
        self.success = False

        if distance < self.epsilon:
            print("Success")
            self.success = True
            self.done = True

        self.step_cnt += 1
        if self.step_cnt > self.max_steps:  # 限制episode长度为step_threshold
            print("MAxStEp")
            self.done = True

        observation = observation

        info = {'is_success': self.success}
        self.cummulative_reward += reward

        def update(arr):
            arr[:] = [action, reward, self.cummulative_reward,
                      self.step_cnt, self.success, distance]
        self.collector.lock_and_modify(update)

        return observation, reward, self.done, info


def make_env(args):
    return gym.make(args.env)


if __name__ == "__main__":
    parser = build_parser()
    args = grab_args(parser)
    args.env = 'RealParkingLot-v0'
    evaluate(args, env_maker=make_env)
