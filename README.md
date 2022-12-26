# smart-car-auto-parking

20 级数科班 2022-2023 年秋冬季大作业

第一阶段成果：基于 ArUco Marker Board 的小车实时位姿测定和可视化（单目）

前置作业：https://github.com/lie-flat/smart-car-deep-learning

B 站视频展示和讲解：

- 单目位姿测定与追踪： https://www.bilibili.com/video/BV1N24y1y7Zt/
- 强化学习自动泊车： TODO

## 环境搭建

### 软件环境搭建

我们假定您已经阅读并理解了前置作业的相关文档。

请在带有 Nvidia 独立显卡的 GNU/Linux 操作系统上运行本项目的代码，我们没有在 Windows 和 Mac 上完整得测试过代码，我们也不支持 Windows / MacOS 操作系统！！！

注意：Python 版本需要大于等于 3.10！

我们建议您使用 Mamba 来创建环境和安装 `pytorch`、`pybullet` 等依赖。注意：不要在 conda/mamba 环境里用 pip 装 pybullet, 会变的不幸。

在克隆此仓库后，请运行 `git submodule update --init` 并确保没有报错，子模块全部同步成功。

建议的配环境过程如下：

创建 Mamba 环境，安装依赖：

```bash
mamba env create -n cv -f environment.yml
mamba activate cv
pip install -r client/requirements.txt
```

后面的操作都假设你处于此 Mamba 环境中

### 手机配置

由于新版安卓在锁屏时会禁用相机，我们需要让安卓手机保持解锁状态。
你可以使用 Wakey 这款软件来做到这一点，部分手机自带了这一功能（比如笔者正在使用的 Lineage OS 19）。

#### 第一阶段

我们需要使用如下软件将安卓手机变成 Web Cam: [DroidCam](https://www.dev47apps.com/)。

去除水印只需要一点简简单单的安卓/Java 逆向工程知识，但是出于尊重知识产权的考虑，
我们不会公开去除水印的版本，你可以购买 DroidCam 高级版来支持原作者。

我写过一篇博客, 讲解如何使用 DroidCam 作为 OpenCV 的视频源： [kxxt 的博客](https://www.kxxt.dev/blog/use-android-devices-as-cameras-in-opencv/)

Android 手机和 Linux 电脑需要在同一个局域网内，请把你的手机 IP 填入 `client/config/platform.py` 中的指定位置。
并将 `PHONE_CAM_MODE` 设置为 `droidcam`

#### 第二阶段

由于 DroidCam 延迟比较高，无法满足我们的自动泊车任务的需求。我们换用 iriun + 有线连接的方式。

在你的手机和电脑上安装 [iriun](https://iriun.com/) 这款软件。

手机上打开 iriun, 将分辨率调节为 640x480. 手机与电脑使用 USB 有线连接。
电脑端打开 iriun, 手机端允许 ADB 调试请求，即可成功连接。

然后将 `client/config/platform.py` 中的 `PHONE_CAM_MODE` 设置为 `iriun`,
`IRIUN_CAM_ID` 设置为 iriun 摄像头的编号。

#### 相机校正

你需要打印一张棋盘格纸进行相机的校正。 相机校正比较基础，不展开讲解。
你可以使用 `client.run.calibration_collector` 这个脚本来收集矫正图片。
然后，你可以运行 `client.run.calibration` 来获得校正参数。

在完成相机校正后，你需要将获得的数据填入 `client/camera/phone.py` 的 `CAMERA_MAT` 和 `DIST_COEFFS` 两个常量中去。

### 小车配置

接线、固件编译烧录的细节操作请翻之前的大作业文档，此处不再赘述。

我们对小车进行了一定的物理改装，来让我们能在顶部平整的粘贴一张 ArUco Marker Board.

你可以运行 `python -m client.run.save_board` 这个命令来保存我们使用的 ArUco Board 的图片到 final-board.png。

然后，你可以把这张图片打印出来（推荐使用激光打印机，不反光，效果更好），贴在小车顶部，请注意让 ArUco Board 保持水平！

![top](resources/images/car-top.jpg)
![front](resources/images/car-front.jpg)

根据小车的实际情况，调整 `client/config/rl.py` 中小车的电机控制值 `REAL_CAR_SPEED` 和 `REAL_CAR_TURN_SPEED`。

### 场景配置

地图平铺，调整支架，保证手机摄像头水平且能拍到整张地图。

<img src="resources/images/setup.jpg" alt="" width="50%">

然后运行 `python -m client.run.cam`， 在显示的图片中所示的圆圈的对应实体地图位置做一个标记。

选定地图的一个角作为世界坐标系原点，计算相机坐标系到世界坐标系的旋转矩阵，
填到 `client/config/positioning.py` 的 `ROTATION` 常量里。

再根据从地图上作的标记，量出相机坐标系与世界坐标系的偏移量（Z 偏移量为摄像头高度），填入 `OFFSET_{X,Y,Z}` 常量中。

## 运行

电脑连接到开发板的热点，同时保证手机和电脑处在同一局域网。

### 第一阶段实时位姿测定的展示(先后启动两个 Python 程序)

```bash
python -m client.run
python -m client
```

### 第二阶段强化学习自动泊车

#### 强化学习虚拟场景调试

我们提供一个交互式的环境，它可以用来进行虚拟场景的调试。

```bash
python -m client.rl.heuristic
```

脚本启动后，你将得到一个 pybullet 窗口和一个 IPython shell.

![heuristic](resources/images/heuristic.png)

你可以在 IPython shell 中自由的执行你想要执行的代码，
进行虚拟场景的调试。要想知道这个 shell 提供了哪些全局变量和函数，请阅读它的源代码。

#### 训练强化学习模型

运行如下命令可以查看训练脚本的使用帮助。

```bash
python -m client.rl.train -h
```

示例

```bash
python -m client.rl.train --model dqn --total-steps 3000000 --init-x=1.5 --init-y=2 --init-theta="np.pi/6" --no-wall --seed=114514
```

你可以启动 tensorboard 来查看训练的情况：

```bash
tensorboard --logdir logs
```

<img src="resources/images/tensorboard.png" alt="tensorboard">

#### 评估强化学习模型

运行如下命令可以查看评估脚本的使用帮助。

```bash
python -m client.rl.train -h
```

示例：

```bash
python -m client.rl.eval --model-path resources/self-parking-nn/dqn_1_1500000.zip  --eval-episodes 10 --render
```

![render](resources/images/render.gif)

#### 部署强化学习模型

我们提供两种部署模式。

- 在数字孪生模式下，我们仍然依赖于 PyBullet 虚拟场景的数据来运行模型，
  即没有使用位姿测定得到的数据来运行模型，
  真实的小车只是简单的跟随虚拟的小车一起做出同步的运动。

- 在真实部署模式下，我们直接把位姿测定得到的数据传递给模型，
  不再依赖 pybullet 虚拟场景，完成本次大作业的最终目标。

##### 数字孪生模式

先后运行以下两个 Python 脚本

```bash
python -m client.run.parking --follow
python -m client.rl.eval --eval-episodes 1 --model-path 模型路径 \
    --init-x=起始X --init-y=起始Y --init-theta=起始theta \
    --render --real --presentation
```

##### 真实部署模式

先后运行以下两个 Python 脚本

```bash
python -m client.run.parking
python -m client.rl.real --model-path 模型路径 --eval-episodes 1
```

## 讲解

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
