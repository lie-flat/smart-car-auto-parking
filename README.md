# smart-car-auto-parking

20 级数科班 2022-2023 年秋冬季大作业

前置作业：https://github.com/lie-flat/smart-car-deep-learning

- 第一阶段成果：基于 ArUco Marker Board 的小车实时位姿测定和可视化（单目）
- 第二阶段成果：基于强化学习方法的自动泊车

B 站视频展示和讲解：

- 单目位姿测定与追踪： https://www.bilibili.com/video/BV1N24y1y7Zt/
- 强化学习自动泊车： TODO

开源代码：

https://github.com/lie-flat/smart-car-auto-parking

## 效果展示

您可以前往 B 站查看更清晰的版本。您也可以在 [`resources/videos` 这个目录下](resources/videos)下载原始视频文件。

| <img src="resources/videos/1.0-full.gif"><br />直线倒车入库<br />完全部署模式                           | <video src="resources/videos/1.0-sim.gif"><br />直线倒车入库<br />数字孪生模式          |
| ------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| <img src="resources/videos/2.120-full.gif"><br />斜后方 120 度倒车入库<br />完全部署模式                | <img src="resources/videos/2.120-sim.gif"><br />斜后方 120 度倒车入库<br />数字孪生模式 |
| <img src="resources/videos/2.120-full2.gif"><br />斜后方 120 度倒车入库<br />完全部署模式（另一次录制） | <img src="resources/videos/3.90-full.gif"><br />斜后方 90 度倒车入库<br />完全部署模式  |

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
mamba env create --file environment.yml
mamba activate park
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

接线细节操作请翻之前的大作业文档，此处不再赘述。

使用 Arduino IDE 打开 `firmware/camera_web_server`, 下载到 ESP32-CAM 上。

使用 VSCode + PlatformIO 打开 `firmware/main`, 构建并下载到开发板上。

开发板的热点的 SSID 是 lie-flat, 密码是 flat-lie

将以下设置写入 `device.cache.json`。

```json
{ "board": "192.168.4.1", "esp32-cam": "192.168.4.2" }
```

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

电脑连接到开发板的热点和手机摄像头。

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

示例:

训练一个 DQN 模型，总步数为 3000000，模型的种子为 114514, 初始位姿的 xy 坐标为 (1.5,2), 初始的旋转为 $\pi\over6$, 不启用墙壁，启用可视化，每 30000 步保存一个 checkpoint, 使用 racecar 汽车模型（默认是 husky），模型放大 2.2 倍：

```bash
python -m client.rl.train --model dqn --total-steps 3000000 --init-x=1.5 --init-y=2 --init-theta="np.pi/6" --no-wall --seed=114514 --render --save-freq=30000 --car=racecar --car-scale=2.2
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

脚本最后会输出 Mean Cummulative Reward 和标准差。

#### 部署强化学习模型

我们提供两种部署模式。

- 在数字孪生模式下，我们仍然依赖于 PyBullet 虚拟场景的数据来运行模型，
  即没有使用位姿测定得到的数据来运行模型，
  真实的小车只是简单的跟随虚拟的小车一起做出同步的运动。

- 在完全部署模式下，我们直接把位姿测定得到的数据传递给模型，
  不再依赖 pybullet 虚拟场景，完成本次大作业的最终目标。

经过多次实验，我们发现真实部署模式比数字孪生模式效果更好，这可能虚拟场景和真实场景的差异造成的。

另外，虽然真实小车的动作的定义与虚拟小车有所不同，我们发现在完全部署模式下，我们的模型仍然能够达到很好的效果。

##### 数字孪生模式

先后运行以下两个 Python 脚本

```bash
python -m client.run.parking --follow
python -m client.rl.eval --eval-episodes 1 --model-path 模型路径 \
    --init-x=起始X --init-y=起始Y --init-theta=起始theta \
    --render --real --presentation
```

##### 完全部署模式

先后运行以下两个 Python 脚本

```bash
python -m client.run.parking
python -m client.rl.real --model-path 模型路径 --eval-episodes 1
```

## Unity 3D 场景

为了更好的还原真实的场景，我们一开始使用 Unity 3D 架设了场景，组建了小车的虚拟模型，
并使用 Unity3D 提供的 ml-agents 训练了强化学习模型。

但是由于无法准确的得知小车轮胎的部分物理参数，
Unity 3D ml-agents 官方也不支持将训练得到的模型部署到 Python 脚本中去，
我们最后放弃了这个计划，转而使用 pybullet + gym + stable_baseline3。

Unity3D 工程的代码在 environment 文件夹下。

![u3d](resources/images/u3d.png)

## 代码讲解

目录结构

```
smart-car-auto-parking
├── client              # Python 客户端， 位姿测定，强化学习自动泊车
├── devices.cache.json  # 设备缓存文件
├── environment         # Unity 3D 工程
├── environment.lock    # mamba 环境配置文件(版本锁定)
├── environment.yml     # mamba 环境配置文件
├── firmware            # C++ 编写的小车/ESP32CAM 固件
├── logs                # Tensorboard 日志文件夹/模型检查点保存
├── measurements        # 手动测量得到的部分参数
├── LICENSE             # 开源协议
├── README.md           # 说明文件
├── resources           # 资源文件夹
├── roadsign            # 交通标志识别模型
└── slides              # 幻灯片
```

### 第一阶段：实时位姿测定与直播推流

您可以查看 [B 站讲解视频](https://www.bilibili.com/video/BV1N24y1y7Zt/) 或讲解幻灯片：

- [位姿测定](https://lie-flat.github.io/smart-car-auto-parking/positioning)
- [直播推流](https://lie-flat.github.io/smart-car-auto-parking/streaming)

### 第二阶段：强化学习自动泊车

#### 小车同步运动 API

之前我们编写的小车的运动 API 是异步的，有状态的，对小车的控制受网络延迟影响大。

这一次我们编写了小车的同步运动 API， 来实现对小车运动的精确控制：

客户端向小车发送请求，让电机和舵机处于指定状态指定长度时间，然后小车停止电机并给客户端返回 `200 OK`

下面是 `firmware/main/src/main.cpp` 的节选:

因为 request handler 在中断里，我们不能阻塞太长时间，否则会触发 watchdog 错误导致重启，所以我们把请求指针塞到一个 FreeRTOS 的队列里去（大小为一即可，我们不支持在上一条同步运动请求完成前执行下一条同步运动请求）。

```c++
QueueHandle_t cmdQueue;

void setup() {
    ...
    cmdQueue = xQueueCreate(1, sizeof(AsyncWebServerRequest*));
    server.on("/act", HTTP_POST, [](AsyncWebServerRequest* request) {
    	// Put the request in the queue
    	if (xQueueSend(cmdQueue, (void*)&request, (TickType_t)10) != pdPASS) {
      		request->send(500);
    	}
  	});
    ...
}
```

然后，我们在 `loop` 里处理请求, 解析出请求参数，调用 `act` 函数，待运动完成后，再给客户端返回 200。

```c++
void loop() {
  if (uxQueueMessagesWaiting(cmdQueue)) {
    AsyncWebServerRequest* request;
    if (xQueueReceive(cmdQueue, &request, (TickType_t)10)) {
      auto duration = parse_int_param(request, "duration");
      auto servo = parse_float_param(request, SERVO_PARAM);
      auto motor_a = parse_float_param(request, MOTOR_A_PARAM);
      auto motor_b = parse_float_param(request, MOTOR_B_PARAM);
      act(servo, motor_a, motor_b, duration);
      request->send(200, "text/plain", "OK");
    }
  }
}
```

`act` 函数调用我们之前写好的 `set_a`,`set_b`,`set_servo` 来控制小车运动, 我们使用 FreeRTOS 的 `vTaskDelay` 来等待 `duration` 毫秒，最后让电机停止运动。

```c++
void act(float servo, float motor_a, float motor_b, int duration) {
  set_servo(servo);
  set_a(motor_a);
  set_b(motor_b);
  vTaskDelay(duration);
  set_a(0);
  set_b(0);
}
```

然后，配套地，我们有如下的 python 库代码(`client/controller/control.py`):

```python
def act(ip, servo=7.5, a=0, b=0, duration=0):
    body = {
        "servo": servo,
        "motorA": a,
        "motorB": b,
        "duration": duration
    }
    requests.post(f"http://{ip}/act", body)
```

#### 配置项

为了方便更改，我们把配置抽出来作为一个 python 模块：`client.config`.

该子模块下有以下配置文件：

- aruco.py: ArUco 检测配置
- boarddef.py： ArUco 板子定义
- common.py：公共设置
- control.py：关键运行模式控制和上下文定义
- misc.py： 杂项
- platform.py：平台相关配置，摄像头运行模式配置
- positioning.py：位姿测定配置
- resources.py：资源配置
- rl.py：强化学习配置

#### 强化学习环境

为了方便，我们把地图等比例放大五倍（以米为单位），地图的 URDF 位于 `resources/ground.urdf`。

我们在 `client/rl/base.py` 中定义了虚拟场景和真实场景的公共基类 `ParkingLotEnvBase`.

这个基类定义了小车的始末位姿、状态空间、误差范围、动作空间、奖励权重、基于共享内存的跨进程数据收集器、距离和奖励的计算函数等。

我们给小车定义了 前进、后退、左转、右转 这四种离散动作（我们在 Unity 3D 环境中采用了连续动作空间）。

然后，我们在 `client/rl/env.py` 中定义了虚拟场景 `ParkingLotEnv`, 该虚拟场景在其基类的基础上增加了 pybullet 仿真的相关功能，实现了一个用来训练和评估强化学习模型的虚拟场景。（代码太长，我不贴了）

为了方便调试虚拟场景，我们还编写了一个脚本： `client/rl/heuristic.py`, 这个脚本会启动 pybullet 场景，同时给你一个 IPython shell 来自由探索，你可以使用 `w`, `a`, `s`, `d` 这四个函数来移动小车，它们接受的参数是运动步数，它们返回结束时的小车的观测值。运行 `reset()` 来重置环境。你也可以调用 pybullet 来随意的修改虚拟场景。

```python
import gym
import pybullet as p
from IPython import embed
from .cmd_parser import build_parser, grab_args
from .impl import make_env
from time import sleep
from math import *
from ..config.rl import *
if __name__ == '__main__':
    parser = build_parser()
    args = grab_args(parser)
    args.render = True
    env = make_env(args)
    env.reset()
    unwrapped = env.unwrapped
    car = unwrapped.car
    reset = env.reset
    def movement_generator(action):
        def f(t=2):
            for _ in range(t):
                obs = env.step(action)
            else:
                return obs
        return f
    w = movement_generator(0)
    s = movement_generator(1)
    a = movement_generator(2)
    d = movement_generator(3)
    embed(header="You are on your own now. Feel free to explore!")
```

另外，我们在 `client/rl/real.py` 中定义了真实场景 `RealParkingLotEnv`, 该虚拟场景在其基类的基础上增加了控制物理小车运动，基于共享内存的环境观测信息收集等功能，实现了用于完全部署模式的真实场景。（代码太长，我不贴了）

我们在 `client/rl/__init__.py` 中注册了上述两个环境，然后我们就可以使用字符串 ID 调用这两个环境了：

```python
from gym.envs.registration import register

register(id='ParkingLot-v0', entry_point='client.rl.env:ParkingLotEnv')
register(id='RealParkingLot-v0', entry_point='client.rl.real:RealParkingLotEnv')
```

小车相关的代码在 `client/rl/car.py` 中，`Car` 类负责了虚拟/真实小车的控制，pybullet 环境小车加载和观测数据收集等功能。

#### 模型训练/评估

`client/rl/cmd_parser.py` 定义了公共的命令行参数解析器。

我们在 `client/rl/models.py` 中对各种模型做了一个抽象，使得我们能够方便的通过命令行参数来切换模型。

模型训练和评估的主要代码在 `client/rl/impl.py` 中。

模型训练：

```python
def train(args):
    env = make_env(args)
    checkpoint_callback = CheckpointCallback(
        save_freq=args.save_freq, save_path=args.ckpt_path, name_prefix=args.model)
    if args.resume_from:
        model_class = get_model_class_by_name(args.model)
        path = Path(args.resume_from)
        if path.is_file():
            model_path = args.resume_from
        else:
            model = str(path/'final.zip')
        model = model_class.load(model_path, env=env)
    else:
        model = init_model_by_name(
            args.model, env=env, verbose=1, seed=args.seed)
    logger = configure(args.log_dir, ["tensorboard"])
    model.set_logger(logger)
    env.reset()
    model.learn(total_timesteps=args.total_steps,
                callback=checkpoint_callback)
    model.save(args.model_path)
    env.close()
```

模型评估：

```python
def evaluate(args, env_maker=make_env):
    env = Monitor(env_maker(args))
    model_class = get_model_class_by_name(args.model)

    path = Path(args.model_path)
    model_path = str(path).removesuffix(
        ".zip") if path.is_file() else str(path/'final')
    model = model_class.load(model_path, env)
    env.reset()
    mean, std = evaluate_policy(
        model, env, n_eval_episodes=args.eval_episodes, render=args.render)
    print(f"{Fore.YELLOW}Mean reward: {mean}, Std: {std}{Style.RESET_ALL}", file=stderr)
```

#### 模型部署

效果展示的脚本位于 `client/run/parking.py`. 出于性能因素考虑，我们没有将效果展示和自动泊车写到一个程序里，我们把它们分到了两个程序中并行运行。

`client/run/parking.py` 实时地测定小车的位置和姿态，并且通过位姿变换把数据变换成观测数据，同时可视化到屏幕上。同时，它通过共享内存的方式读取自动泊车的相关信息，一并显示在屏幕上。在完全部署模式下，该脚本还会把观测数据通过共享内存的方式传递回自动泊车脚本。代码太长了，我就不贴了。

`client/rl/analytics.py` 封装了收集自动泊车信息的类 `AnalyticsCollector/AnalyticsReader`:

我们使用文件锁作为两个进程读写共享内存的互斥量来防止脏读（Dirty Read）问题的出现。在大作业的第一阶段我们并没有类似的机制来防止脏读，这是因为第一阶段共享内存的数据为图像数据，脏读图像并无不良影响，几乎不影响图像的呈现。

```python
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
```

## Reference

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
