
import numpy as np
from os import path
from .config import detModelDir, paddleDetDir
import sys
import cv2

sys.path.append(paddleDetDir)
sys.path.append(path.join(paddleDetDir, 'deploy', 'python'))

from infer import Detector, preprocess, Resize, NormalizeImage, Permute, PadStride, LetterBoxResize, WarpAffine, Pad

class DetModel:
    def __init__(self):
        self.detector = Detector(detModelDir, device='GPU')
        self.preprocess_ops = []
        for op_info in self.detector.pred_config.preprocess_infos:
            new_op_info = op_info.copy()
            op_type = new_op_info.pop('type')
            self.preprocess_ops.append(eval(op_type)(**new_op_info))
        self.input_names = self.detector.predictor.get_input_names()
        self.output_names = self.detector.predictor.get_output_names()

    def predict(self, img):
        img, img_info = preprocess(img, self.preprocess_ops)
        inputs = {}
        inputs['image'] = np.array((img, )).astype('float32')
        inputs['im_shape'] = np.array(
            (img_info['im_shape'], )).astype('float32')
        inputs['scale_factor'] = np.array(
            (img_info['scale_factor'], )).astype('float32')
        for i in range(len(self.input_names)):
            input_tensor = self.detector.predictor.get_input_handle(self.input_names[i])
            input_tensor.copy_from_cpu(inputs[self.input_names[i]])
        self.detector.predictor.run()
        boxes_tensor = self.predictor.get_output_handle(self.output_names[0])
        np_boxes = boxes_tensor.copy_to_cpu()
        boxes_num = self.predictor.get_output_handle(self.output_names[1])
        np_boxes_num = boxes_num.copy_to_cpu()
        result = dict(boxes=np_boxes, boxes_num=np_boxes_num)
        return result


if __name__ == '__main__':
    img = cv2.imread('../2022-08-06-18-17-42.jpg')
    det = DetModel()
    result = det.predict('../2022-08-06-18-17-42.jpg' )
    print(result)