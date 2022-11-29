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

# 单目相机位姿测定与追踪

山东大学（威海） 2020 级 数据科学与人工智能实验班

GitHub: https://github.com/lie-flat/smart-car-auto-parking

---
layout: two-cols
---

# 项目目录结构

```
resources
├── SmileySans.LICENSE
└── SmileySans-Oblique.ttf
client
├── ai（之前大作业原有文件略）
├── camera
│   ├── 之前大作业原有文件略
│   ├── __init__.py
│   └── phone.py
├── config
│   ├── aruco.py
│   ├── boarddef.py
│   ├── control.py
│   ├── __init__.py
│   ├── misc.py
│   ├── platform.py
│   ├── positioning.py
│   └── resources.py
├── controller（之前大作业原有文件略）

```

::right::

```
client
├── cv
│   ├── aruco.py
│   ├── cat.py
│   ├── __init__.py
│   └── 之前大作业原有文件略
├── __init__.py
├── __main__.py
└── run
    ├── calibration_collector.py
    ├── calibration.py
    ├── cam.py
    ├── draw_charuco.py
    ├── __init__.py
    ├── __main__.py
    ├── marker.py
    ├── save_board.py
    ├── takepics.py
    └── test_cat.py
```

---

# 相机标定
client/run/calibration_collector.py

收集图像：若当前图像中能标定到棋盘格，就保存该图像。然后我们使用 client/run/calibration.py 即可得到相机参数。
（也可以使用 ZhangZhengyou Method, 参考网上代码）


```python
while True:
    ret, img = vid.read()
    if not ret:
        raise Exception("Failed to read image!")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (10, 7), None)
    if ret == True:
        print("FOUND")
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        cv.imwrite(f"cab/{i}.det.png", img)
        cv.drawChessboardCorners(img, (10, 7), corners2, ret)
        cv.imshow("I", img)
        i += 1
        if cv.waitKey(2) & 0xff == ord('q'):
            break
```


---

# 基于 ArUco Marker Board 的位姿测定
client/cv/aruco.py

```python
import cv2.aruco as aruco
# 常量的引入略
def estimate_pose_and_draw(frame):
    corners, ids, _rejected_points = aruco.detectMarkers(frame, dic)
    rotation_world = None
    rotation = None
    translation = None
    translation_world = None
    if ids is not None and len(ids) >= 4:
        aruco.drawDetectedMarkers(frame, corners, ids)
        if DETECT_BOARD:
            valid_cnt, rotation, translation = aruco.estimatePoseBoard(
                corners, ids, BOARD, CAMERA_MAT, DIST_COEFFS, None, None)
            if valid_cnt > 0:
                cv.drawFrameAxes(frame, CAMERA_MAT, DIST_COEFFS,
                                 rotation, translation, 0.08, 6)
            rotation_camera, _ = cv.Rodrigues(rotation)
            rotation_world = ROTATION @ rotation_camera
            translation_world = ROTATION @ translation + \
                TRANSLATION.reshape(3, 1)
    return frame, rotation, translation, rotation_world, translation_world
```

---

# 可视化
client/cv/cat.py

这是直播流底部固定文字的绘制代码。（因为 cv.putText 不支持中文字符，所以我们在这里使用 Pillow 来绘制文字）

```python
TEXT_AREA = np.ones((TEXT_AREA_HEIGHT, TEXT_AREA_WIDTH, 3),
                    dtype=np.uint8) * 255
img_pil = Image.fromarray(TEXT_AREA)
draw = ImageDraw.Draw(img_pil)
draw.text((10, -3),  "求个 Star, 谢谢喵~: https://github.com/lie-flat/smart-car-auto-parking",
          font=CHINESE_FONT, fill=(0xFF, 0x90, 0x1E))
draw.text((1260, -3),  "非常感谢得意黑 SmileySans 这款开源字体",
          font=CHINESE_FONT, fill=(0x75, 0x7A, 0x0B))
draw.text((10, 45),  "*: 因为 ESP32 CAM 网络延迟问题，小车摄像头的画面有时会有不确定的延迟（一般在 1s 左右）",
          font=CHINESE_FONT, fill=(0x4B, 0x4B, 0xE5))
draw.text((1500, 45),  "山东大学（威海）,数科班",
          font=CHINESE_FONT, fill=(0xC5, 0xFF, 0x00))
TEXT_AREA = np.array(img_pil)
```

---

# 可视化
client/cv/cat.py:func cat

- 清空信息展示区和位姿可视化区。
- 转换输入图像的色彩空间，进行缩放之后
- 处理此帧没有定到位姿的情况，显示 NaN

```python
global p0x, p1x, p2x, p3x, p0y, p1y, p2y, p3y, traj, visual
info_area.fill(255)
visual.fill(255)
# Resize inputs
road_mask = cv.resize(road_mask, (PHONE_CAM_WIDTH, PHONE_CAM_HEIGHT))
road_perspective = cv.cvtColor(road_perspective, cv.COLOR_GRAY2BGR)
road_perspective = cv.resize(
    road_perspective, (PHONE_CAM_WIDTH, PHONE_CAM_HEIGHT))
# Null coalescing
world_trans = null_coalesce(world_trans, PLACEHOLDER)
cam_trans = null_coalesce(cam_trans, PLACEHOLDER)
cam_rot = null_coalesce(cam_rot, PLACEHOLDER)
```

---

# 可视化
client/cv/cat.py:func cat

若测定出了位姿, 计算小车的中心点的世界坐标，在可视化区画出表示位姿的矩形。同时在轨迹图上画一个点。

```python
if world_rot is not None:
    cost, sint = world_rot[0, 0], world_rot[1, 0]
    p0x, p0y = world_trans[1].item(), world_trans[0].item()
    p1x = int((p0x + CAR_WIDTH * cost) * MAP_FACTOR)
    p1y = int((p0y - CAR_WIDTH * sint) * MAP_FACTOR)
    p3x = int((p0x + CAR_HEIGHT * sint) * MAP_FACTOR)
    p3y = int((p0y + CAR_HEIGHT * cost) * MAP_FACTOR)
    p0x, p0y = int(MAP_FACTOR*p0x), int(MAP_FACTOR*p0y)
    deltax, deltay = p1x - p0x, p1y - p0y
    p2x, p2y = p3x + deltax, p3y + deltay
    cv.line(visual, (p0x, p0y), (p1x, p1y), (0, 255, 0), RECT_BORDER_THICKNESS)
    cv.line(visual, (p0x, p0y), (p3x, p3y), (0, 0, 255), RECT_BORDER_THICKNESS)
    cv.line(visual, (p1x, p1y), (p2x, p2y), (255, 0, 0), RECT_BORDER_THICKNESS)
    cv.line(visual, (p3x, p3y), (p2x, p2y), (0, 0, 0), RECT_BORDER_THICKNESS)
    pos = (int((p0x + p2x)/2), int((p0y+p2y)/2))
    visual = cv.circle(visual, pos, 4, (0x6E, 0x00, 0xFF), 4)
    # Convert mats to vecs
    world_rot, _ = cv.Rodrigues(world_rot)
    traj = cv.circle(traj, pos, 4, (0x6E, 0x00, 0xFF), 6)
else: world_rot = PLACEHOLDER
```