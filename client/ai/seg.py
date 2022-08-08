import numpy as np
from os import path
from .config import segModelDir, paddleSegDir
import paddle.inference as paddle_infer
import sys
import cv2
from PIL import Image


sys.path.append(paddleSegDir)
from paddleseg.transforms import Normalize, Compose
from paddleseg.utils.visualize import get_color_map_list


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
        self._color_map = get_color_map_list(256)

    def predict(self, img: np.ndarray) -> np.ndarray:
        img = self._transform({'img': img.astype('float32')})['img'][np.newaxis, ...]
        self._input_handle.reshape(img.shape)
        self._input_handle.copy_from_cpu(img)
        self.predictor.run()
        return self._output_handle.copy_to_cpu()
    
    def colormap(self, pred: np.ndarray) -> np.ndarray:
        pred_mask = Image.fromarray(pred.astype(np.uint8).squeeze(), mode='P')
        pred_mask.putpalette(self._color_map)
        return np.array(pred_mask.convert('RGB'))[:, :, ::-1] # To BGR

if __name__ == '__main__':
    img = cv2.imread("../2022-08-06-18-17-07.jpg")
    model = SegModel()
    pred = model.predict(img)
    colormap = model.colormap(pred) 
    cv2.imshow("colormap", colormap)
    cv2.waitKey(0)
    print(colormap)