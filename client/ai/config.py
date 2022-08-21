from os import path

fileDir = path.dirname(__file__)
segModelDir = path.join(fileDir, '..', '..', 'roadseg', 'models')
detModelDir = path.join(fileDir, '..', '..', 'roadsign')
paddleSegDir = path.join(fileDir, 'PaddleSeg')
paddleDetDir = path.join(fileDir, 'PaddleDetection')
