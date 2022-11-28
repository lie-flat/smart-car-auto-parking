import numpy as np
from ..config import IS_RASPBERRYPI

if IS_RASPBERRYPI:
    CAMERA_MAT = np.array([[621.98474404,   0., 325.83155255],
                           [0., 622.99381138, 242.90768789],
                           [0.,   0.,   1.]], dtype="float32")
    DIST_COEFFS = np.array(
        [[0.18640865, -0.83050602, -0.00315513,  0.00577651,  0.91716078]], dtype="float32")
else:
    CAMERA_MAT = np.array([[470.75829554,   0., 320.85206853],
                           [0., 472.51686419, 242.5783875],
                           [0.,   0.,   1.]], dtype="float32")
    DIST_COEFFS = np.array(
        [[0.04556832, -0.10639779,  0.00078644,  0.00124518,  0.08526551]], dtype="float32")
