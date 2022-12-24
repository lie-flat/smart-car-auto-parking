# smart-car-auto-parking

20 级数科班 2022-2023 年秋冬季大作业

第一阶段成果：基于 ArUco Marker Board 的小车实时位姿测定和可视化（单目）

前置作业：https://github.com/lie-flat/smart-car-deep-learning

我们假定您已经阅读并理解了前置作业的相关文档。

注意：Python 版本需要大于等于 3.10！为了提高性能，请在配好环境之后确认 PyBullet 启用了 Numpy 支持（`pybullet.isNumpyEnabled() == 1`）。

需要使用如下软件将安卓手机变成 Web Cam: [DroidCam](https://www.dev47apps.com/)。
我写过一篇博客，这里不再赘述： [kxxt 的博客](https://www.kxxt.dev/blog/use-android-devices-as-cameras-in-opencv/)

B 站视频展示和讲解： https://www.bilibili.com/video/BV1N24y1y7Zt/

# 小车

接线、固件编译烧录请翻之前的大作业文档，此处不再赘述。

我们对小车进行了一定的物理改装，来让我们能在顶部平整的粘贴一张 Aruco Marker Board.

# 讲解

您可以看 B 站视频。若您像 kxxt 一样更 prefer 文字版的讲解，也可以看 slides 目录下的讲解幻灯片。

# Reference

- https://markhedleyjones.com/projects/calibration-checkerboard-collection
- https://calib.io/pages/camera-calibration-pattern-generator
- https://docs.opencv.org/4.6.0/d5/dae/tutorial_aruco_detection.html
- https://docs.opencv.org/4.6.0/db/da9/tutorial_aruco_board_detection.html
- https://docs.opencv.org/4.6.0/df/d4a/tutorial_charuco_detection.html
- https://docs.opencv.org/4.6.0/d1/dcb/tutorial_aruco_faq.html
- https://github.com/RyanLiu112/RL_parking
- https://github.com/Robotics-Club-IIT-BHU/gym-carpark
- https://github.com/VanIseghemThomas/AI-Parking-Unity
- https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.gpdptdmpokh
- 