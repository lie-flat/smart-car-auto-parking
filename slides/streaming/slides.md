---
# try also 'default' to start simple
theme: seriph
# random image from a curated Unsplash collection by Anthony
# like them? see https://unsplash.com/collections/94734566/slidev
background: bg.jpg
# apply any windi css classes to the current slide
class: "text-center"
# https://sli.dev/custom/highlighters.html
highlighter: shiki
# show line numbers in code blocks
lineNumbers: false
drawings:
  persist: false
# use UnoCSS
css: unocss
---

# 单目相机位姿测定与追踪

山东大学（威海） 2020 级 数据科学与人工智能实验班

GitHub: https://github.com/lie-flat/smart-car-auto-parking

---

# 目录

这里是目录

- 直播推流导致的严重掉帧问题
- 不进行推流时，程序正常运行的效果展示（因为视频比较短，这部分重复五次）
- 昨天的直播回放（由于技术原因，没存下来最后那一部分）
- 整体网络架构
- 推流实现
- 实时位姿测定与追踪实现

---
layout: cover
background: bg.jpg
---

# 直播推流导致的严重掉帧问题

---

# 直播推流导致的严重掉帧问题

。

<v-clicks>

- 我们在直播推流时发现我们原本运行的流畅无比的程序变的异常卡顿
- FPS 经常掉到 0， 导致整个直播卡的不堪入目
- 后来，经过我们的排查，我们发现只要我们把 B 站的直播推流关掉，我们的程序就能顺畅的运行了。
- 因此，我们断定 B 站直播推流出于某些奇怪的技术原因会导致我们的程序严重掉帧
- 我们对此次糟糕的直播体验深感抱歉
- 接下来，我们先来看一下程序正常运行时的录像

</v-clicks>

---
layout: cover
background: bg.jpg
---

# 程序正常运行的效果展示

## 因为视频很短，所以此处重复五次播放

---
layout: cover
background: bg.jpg
---

# 直播回放

## 惨不忍睹

---
layout: cover
background: bg.jpg
---

# 整体网络架构

---
layout: two-cols
---
# 整体网络架构
这混乱邪恶的网络架构

<img src="/network.svg" alt="" style="height: 400px;">

::right::

- 因为我们让电脑开热点并让小车连接电脑热点时，从 ESP32 读图像会非常慢，所以我们选择让开发板开热点。
- 作为俯拍摄像头的安卓手机在连接开发板的热点时，电脑访问不到手机的摄像头。。。我不知道是什么问题。或许可怜的 ESP 32 开发板没有想到它那么羸弱的无线功能真的会被人拿来狠狠的当正经局域网用。
- 所以我拿电脑开了个热点给手机用.(注意，这里不能用手机开热点让电脑连接，因为我的电脑只有一个无线网卡，且已经连接到开发板的热点)
- 所以现在我们的电脑没有联网，无法直播。那怎么拌呢？北衡楼 17 楼空地没有网线接口。我们只好找了另一部手机，
使用在 2022 年仍然不过时的 USB 2.0 协议连接到电脑，让手机给电脑 USB 共享网络。
- 万事大吉，我们终于可以开播了。（在这里特别吐槽一下某些手机厂商不给手机上 USB 3.0 的做法）

---
layout: cover
background: bg.jpg
---
# 直播推流的实现

---

# 直播推流的实现
B 站推流

我们在 OpenCV 里创建了一个全屏显示的无边框窗口。

```python
cv.namedWindow("frame", cv.WINDOW_NORMAL)
cv.setWindowProperty("frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
```

我们把手机摄像头俯拍的画面，小车摄像头的画面叠加上道路分割的蒙板，分割出来的道路做过视角变换后的可视化，全局地图上小车轨迹的可视化、全局地图上小车当前位姿的可视化、文字形式的小车当前位姿信息、当前 FPS 还有部分宣传文字都放到这一个全屏画布上。

然后在 B 站直播页面选择直播源为当前整个屏幕，即可向 B 站推流。

这样同时解决了一个问题：

我们的 FPS 是一直在变的，如果直接将 OpenCV 的流推给 B 站，会导致我们的直播流有时快，有时慢。

而我们选择将当前屏幕作为直播流的话就不会出现这个问题。

---
layout: two-cols
---
# 直播推流的实现
快慢视频源的整合

众所周知，AI Thinker ESP32 Cam 是一款物美价廉的物联网摄像头。我们在电脑上通过网络请求读取到这个摄像头的一帧画面需要花上 0.1s 到 1s 的时间。

如果我们把所有的代码都放到一个 Python 进程里，我们的直播的刷新率会不可避免的被这个廉价摄像头卡脖子（Bottleneck）。

为了解决这个问题，我们创新性的将代码拆分成两部分。上次大作业的代码负责了道路分割和交通标志识别等功能，我们把它放到 `client.run.__main__` 中。这次大作业的代码负责了小车位姿测定和追踪和 OpenCV 画面拼接等功能，我们把它放到 `client.__main__` 中。

我们让上一次大作业的程序通过共享内存（Shared Memory) 的方式把需要展示的画面与本次大作业的程序共享。

::right::

### `client.__main__`

```python

img_result_shm = SharedMemory(name=SHM_IMG_RESULT_NAME)
img_result = np.ndarray(IMG_RESULT_SHAPE, dtype=SHM_NP_DTYPE,
                        buffer=img_result_shm.buf)
img_warp_shm = SharedMemory(name=SHM_IMG_WARP_NAME)
img_warp = np.ndarray(IMG_WARP_SHAPE, dtype=SHM_NP_DTYPE,
                      buffer=img_warp_shm.buf)

```

- 共享内存的读取方
- 遵循软件工程的原理，我们把读取方和写入方的公共参数（如 `IMG_WARP_SHAPE`、`SHM_NP_DTYPE`、 `SHM_IMG_RESULT_NAME`）重构进了一个子模块中。
- 从裸共享内存构造 Numpy 数组，这使得我们能直接在拼接直播流时像使用正常 Numpy 数组一样使用它。

---
layout: two-cols
---
# 直播推流的实现
快慢视频源的整合

在我们将一个程序拆分成两个程序之后。我们也就从单进程转向了多进程。（由于 Global Interpreter Lock，多线程 Python 程序并不能合理利用多核处理器的优势）

程序拆分之后，我们的整个直播流的帧率从大约 1 FPS 飙升到了大约 20 FPS,最高的时候能到 40 FPS. 也就是说，我们通过多进程将帧率提升了 19 倍。

多进程解决了快视频源（手机摄像头和后续的位姿测定可视化）和慢视频源（ESP32CAM 及后续的道路分割、交通标志检测）同步处理时的直播流刷新率被限制到大概 1 FPS 左右的问题。

同时，多进程也使得（道路分割和交通标志检测）与（小车实时位姿测定与追踪）能够同时进行，极大地提高了程序的效率。

::right::

### `client.run.__main__`

```python
def create_shared_memory_nparray(data, name):
    d_size = np.dtype(SHM_NP_DTYPE).itemsize * np.prod(data.shape)
    shm = SharedMemory(create=True, size=d_size, name=name)
    dst = np.ndarray(shape=data.shape, dtype=SHM_NP_DTYPE, buffer=shm.buf)
    dst[:] = data[:]
    return shm
img_result_shm = create_shared_memory_nparray(np.zeros(
IMG_RESULT_SHAPE, dtype=SHM_NP_DTYPE), SHM_IMG_RESULT_NAME)
img_warp_shm = create_shared_memory_nparray(np.zeros(
    IMG_WARP_SHAPE, dtype=SHM_NP_DTYPE), SHM_IMG_WARP_NAME)
img_result = np.ndarray(shape=IMG_RESULT_SHAPE, \
    dtype=SHM_NP_DTYPE, buffer=img_result_shm.buf)
img_warp = np.ndarray( shape=IMG_WARP_SHAPE, \
    dtype=SHM_NP_DTYPE, buffer=img_warp_shm.buf)
```

- 写入方的共享内存代码
- 创建共享内存的函数
  - 使用 Python 标准库创建共享内存
  - 从裸共享内存构造 Numpy 数组，将初始数据写入进去。
- 调用上面的函数创建两块共享内存，创建对应的数组

---

# 直播推流的实现
直播流的拼接

```python

def cat(phone_cam, road_mask, road_perspective, world_trans, world_rot, cam_trans, cam_rot, fps):
    """
            640           640               640
         +------------+-------------+-------------------+
      4  | marker det | road mask   | road(perspective) |
      8  | i:480x640  | i:240x320   | i:240x320x1       |
      0  | 480x640    | 480x640     | 480x640           |
         +------------+-+------------+-+----------------+
      5  | trajectory |S|rect visual |S|info display    |
      0  | i: 507x605 |1|i: 507x605  |1|xyz             |
      7  | o: 507x605 |0|o: 507x605  |0|rotation:       |
         +------------+-+------------+-+-----------------+
      93 | text                                         |
         +----------------------------------------------+
    """
    ...
```

图中的 ASCII Art 直观地显示了拼接的方式。

---

video_buffer 是一个全局变量，存储当前直播流画面。

我们在把各种可视化做完之后，将它们通过切片赋值的方式写入到当前直播流画面中。

之前在这个地方我们没有用全局变量和切片赋值. 我们之前用 `np.hstack` 和 `np.vstack` 将各个部分拼起来，这会导致在每一帧中都有不必要的内存分配/释放，实测效率比现在的实现要低。

```python
# global scope
video_buffer = np.ones((VIDEO_HEIGHT, VIDEO_WIDTH, 3),
                       dtype=np.uint8) * 255
# In func cat
video_buffer[:PHONE_CAM_HEIGHT, :PHONE_CAM_WIDTH] = cv.flip(phone_cam, 1)
video_buffer[:PHONE_CAM_HEIGHT,
              PHONE_CAM_WIDTH:2 * PHONE_CAM_WIDTH] = road_mask
video_buffer[:PHONE_CAM_HEIGHT, 2*PHONE_CAM_WIDTH:] = road_perspective
video_buffer[PHONE_CAM_HEIGHT:PHONE_CAM_HEIGHT +
              MAP_LEN_X, :MAP_LEN_Y] = traj
video_buffer[PHONE_CAM_HEIGHT:PHONE_CAM_HEIGHT +
              MAP_LEN_X, MAP_LEN_Y:MAP_LEN_Y + SEPARATOR_WIDTH] = SEPARATOR
```

