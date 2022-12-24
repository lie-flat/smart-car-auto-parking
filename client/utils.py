from multiprocessing.shared_memory import SharedMemory
import numpy as np


def create_shared_memory_nparray(data, name, dtype):
    d_size = np.dtype(dtype).itemsize * np.prod(data.shape)
    shm = SharedMemory(create=True, size=d_size, name=name)
    dst = np.ndarray(shape=data.shape, dtype=dtype, buffer=shm.buf)
    dst[:] = data[:]
    return shm
