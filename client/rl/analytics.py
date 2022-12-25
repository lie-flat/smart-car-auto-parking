from multiprocessing.shared_memory import SharedMemory
import numpy as np
from filelock import FileLock

from ..config.rl import ENVINFO_SHM_NAME, ENVINFO_DTYPE, ENVINFO_FILELOCK_PATH, ENVINFO_SIZE
from ..utils import create_shared_memory_nparray


class AnalyticsCollector:
    """
    Collect analytic info into shared memory
    """

    def __init__(self):
        self.shm = SharedMemory(name=ENVINFO_SHM_NAME)
        self.array = np.ndarray(ENVINFO_SIZE, dtype=ENVINFO_DTYPE,
                                buffer=self.shm.buf)
        self.lock = FileLock(ENVINFO_FILELOCK_PATH)

    def lock_and_modify(self, f):
        with self.lock:
            f(self.array)


class AnalyticsReader:
    def __init__(self) -> None:
        self.shm = create_shared_memory_nparray(
            np.zeros(ENVINFO_SIZE, dtype=ENVINFO_DTYPE), ENVINFO_SHM_NAME, ENVINFO_DTYPE)
        self.array = np.ndarray(ENVINFO_SIZE, dtype=ENVINFO_DTYPE,
                                buffer=self.shm.buf)
        self.lock = FileLock(ENVINFO_FILELOCK_PATH)

    def read_to_dict(self, out_dict):
        with self.lock:
            out_dict["last_action"] = int(self.array[0])
            out_dict["last_reward"] = self.array[1]
            out_dict["cummulative_reward"] = self.array[2]
            out_dict["step_counter"] = int(self.array[3])
            out_dict["success"] = int(self.array[4]) == 1
            out_dict["distance"] = self.array[5]
