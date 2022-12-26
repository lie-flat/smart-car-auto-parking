---
# try also 'default' to start simple
theme: seriph
# random image from a curated Unsplash collection by Anthony
# like them? see https://unsplash.com/collections/94734566/slidev
background: bg.jpg
# apply any windi css classes to the current slide
class: 'text-center'
# https://sli.dev/custom/highlighters.html
highlighter: shiki
# show line numbers in code blocks
lineNumbers: false
# persist drawings in exports and build
drawings:
  persist: false
# use UnoCSS
css: unocss
---

# 强化学习自动泊车 效果展示

山东大学（威海） 2020 级 数据科学与人工智能实验班

GitHub: https://github.com/lie-flat/smart-car-auto-parking


---

# 强化学习自动泊车

两种运行模式

我们使用 PyBullet + stable_baselines3 + gym 训练了小车在虚拟场景中，
从多个位置和姿态出发的自动泊车任务，我们提供两种将模型部署到实际小车上的模式。

- 在数字孪生模式下，我们仍然依赖于 PyBullet 虚拟场景的数据来运行模型，
  即没有使用位姿测定得到的数据来运行模型，
  真实的小车只是简单的跟随虚拟的小车一起做出同步的运动。

- 在完全部署模式下，我们直接把第一阶段的位姿测定得到的数据进行坐标变换后传递给模型，
  不再依赖 pybullet 虚拟场景，完成本次大作业的最终目标。

经过多次实验，我们发现真实部署模式比数字孪生模式效果更好，这可能虚拟场景和真实场景的差异造成的。

另外，虽然真实小车的动作的定义与虚拟小车有所不同，我们发现在完全部署模式下，我们的模型仍然能够达到很好的效果。

---
layout: cover
background: bg.jpg
---

# 直线倒车入库

## 数字孪生模式

---
layout: cover
background: bg.jpg
---

# 直线倒车入库

## 完全部署模式

---
layout: cover
background: bg.jpg
---

# 斜后方 120 度倒车入库

## 数字孪生模式

### 因为忘记切换展示模式了，视频下方显示的是完全部署模式，其实是数字孪生模式

---
layout: cover
background: bg.jpg
---

# 斜后方 120 度倒车入库

## 完全部署模式

---
layout: cover
background: bg.jpg
---

# 斜后方 120 度倒车入库

## 完全部署模式（另一次录制）

---
layout: cover
background: bg.jpg
---

# 斜后方约 90 度倒车入库

## 完全部署模式