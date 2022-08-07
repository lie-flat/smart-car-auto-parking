import numpy as np
from os import path
from .config import segModelDir, paddleSegDir
import paddle.inference as paddle_infer
import sys
import cv2

sys.path.append(paddleSegDir)
from paddleseg.transforms import Normalize, Compose


class SegModel:
    def __init__(self) -> None:
        config = paddle_infer.Config(
            path.join(segModelDir, 'model.pdmodel'),
            path.join(segModelDir, 'model.pdiparams'))
        self.predictor = paddle_infer.create_predictor(config)
        input_names = self.predictor.get_input_names()
        self._input_handle = self.predictor.get_input_handle(input_names[0])
        output_names = self.predictor.get_output_names()
        self._output_handle = self.predictor.get_output_handle(output_names[0])
        self._transform = Compose([Normalize()])

    def predict(self, img: np.ndarray) -> np.ndarray:
        img = self._transform({'img': img.astype('float32')})['img'][np.newaxis, ...]
        self._input_handle.reshape(img.shape)
        self._input_handle.copy_from_cpu(img)
        self.predictor.run()
        return self._output_handle.copy_to_cpu()

if __name__ == '__main__':
    img = cv2.imread("../2022-08-06-18-17-07.jpg")
    model = SegModel()
    print(model.predict(img))