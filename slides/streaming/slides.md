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

# 单目相机位姿测定与追踪 展示

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

<img src="network.svg" alt="" style="height: 400px;">

::right::

- 因为我们让电脑开热点并让小车连接电脑热点时，从 ESP32 读图像会非常慢，所以我们选择让开发板开热点。
- 作为俯拍摄像头的安卓手机在连接开发板的热点时，电脑访问不到手机的摄像头。。。我不知道是什么问题。或许可怜的 ESP 32 开发板没有想到它那么羸弱的无线功能真的会被人拿来狠狠的当正经局域网用。
- 所以我拿电脑开了个热点给手机用.(注意，这里不能用手机开热点让电脑连接，因为我的电脑只有一个无线网卡，且已经连接到开发板的热点)
- 所以现在我们的电脑没有联网，无法直播。那怎么拌呢？北衡楼 17 楼空地没有网线接口。我们只好找了另一部手机，
使用在 2022 年仍然不过时的 USB 2.0 协议连接到电脑，让手机给电脑 USB 共享网络。
- 万事大吉，我们终于可以开播了。（在这里特别吐槽一下某些手机厂商不给手机上 USB 3.0 的做法）