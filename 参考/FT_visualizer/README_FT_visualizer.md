# FT 雷达数据 RViz 可视化工具

基于 ROS2 + RViz 的雷达点云与航迹数据逐帧回放工具，支持俯视（XY）和侧视（XZ）两种视角，提供完整版（含航迹）和简化版（纯点云）两套脚本。

---

## 目录

- [文件说明](#文件说明)
- [环境准备](#环境准备)
- [数据准备](#数据准备)
- [脚本配置](#脚本配置)
- [启动方式](#启动方式)
- [RViz 配置](#rviz-配置)
- [参数速查](#参数速查)

---

## 文件说明

| 文件 | 用途 | 所需数据 |
|------|------|----------|
| `rviz_FT_visualizer_xy.py` | 完整版·俯视图（XY） | 点云 CSV + 航迹 Trks CSV |
| `rviz_FT_visualizer_xz.py` | 完整版·侧视图（XZ） | 点云 CSV |
| `rviz_FT_visualizer_xy_simple.py` | 简化版·俯视图（XY） | 简化点云 CSV（仅 4 列） |
| `rviz_FT_visualizer_xz_simple.py` | 简化版·侧视图（XZ） | 简化点云 CSV（仅 4 列） |
| `launch_visualizers.py` | 并行启动多个可视化脚本 | — |
| `requirements.txt` | Python 依赖列表 | — |

---

## 环境准备

### 1. 安装 ROS2

本工具依赖 ROS2（推荐 Humble 或 Iron）。请参考官方文档完成安装：
https://docs.ros.org/en/humble/Installation.html

安装完成后，每次打开终端需 source 环境：

```bash
# Linux
source /opt/ros/humble/setup.bash

# Windows（PowerShell）
C:\dev\ros2_humble\local_setup.ps1
```

### 2. 创建 Python 虚拟环境（可选但推荐）

```bash
# 创建虚拟环境
python -m venv .venv

# 激活（Linux/macOS）
source .venv/bin/activate

# 激活（Windows PowerShell）
.venv\Scripts\Activate.ps1
```

### 3. 安装 Python 依赖

```bash
pip install -r requirements.txt
```

依赖清单：

| 包 | 最低版本 | 用途 |
|----|----------|------|
| numpy | 1.21.0 | 点云数组运算 |
| pandas | 1.3.0 | CSV 文件读取 |
| opencv-python | 4.5.0 | 高度色带图像生成、视频解码 |

> ROS2 相关包（`rclpy`、`sensor_msgs`、`visualization_msgs`、`cv_bridge`、`tf2_ros`）随 ROS2 安装，无需通过 pip 安装。

---

## 数据准备

### 完整版数据格式（`rviz_FT_visualizer_xy.py` / `rviz_FT_visualizer_xz.py`）

需要两个 CSV 文件：

**点云文件**（必须包含以下列，列名需完全匹配）：

| 列名 | 类型 | 说明 |
|------|------|------|
| `frameID` | int | 帧编号 |
| `xpos` | float | 前向距离（m） |
| `ypos` | float | 横向距离（m） |
| `zpos` | float | 高度（m） |
| `assoTrkID` | int | 关联航迹 ID |
| `DOAMethod` | int | DOA 方法标志（1 = 有效） |

**航迹文件**（仅 xy 完整版需要，必须包含以下列）：

| 列名 | 类型 | 说明 |
|------|------|------|
| `frameID` | int | 帧编号 |
| `objID` | int | 目标 ID |
| `objXPos` | float | 目标前向位置（m） |
| `objYPos` | float | 目标横向位置（m） |
| `objBoxLength` | float | 目标框长度（m） |
| `objBoxWidth` | float | 目标框宽度（m） |
| `f32ObjBoxHeight` | float | 目标框高度（m） |
| `headingAngle` | float | 航向角（rad） |

可选列（存在时自动使用）：`objBoxCenterLgt`、`objBoxCenterLat`（框中心纵/横向偏移）

---

### 简化版数据格式（`rviz_FT_visualizer_xy_simple.py` / `rviz_FT_visualizer_xz_simple.py`）

只需一个 CSV 文件，**按列位置读取，列名不限**：

| 列位置 | 对应字段 | 说明 |
|--------|----------|------|
| A（第 1 列） | frameID | 帧编号 |
| B（第 2 列） | xpos | 前向距离（m） |
| C（第 3 列） | ypos | 横向距离（m） |
| D（第 4 列） | zpos | 高度（m） |

第 5 列及之后的列会被忽略。示例：

```
frame,x,y,z
1,10.5,2.3,-0.1
1,11.2,1.8,0.2
2,10.8,2.1,-0.05
```

### 同步视频文件（仅 `rviz_FT_visualizer_xy_simple.py` 支持）

视频格式为 `.avi`（`cv2.VideoCapture` 可自动识别大多数常见格式）。视频将与点云严格同步播放：总播放时间由 CSV 总帧数 ÷ 播放帧率确定，视频帧率自动计算以保证两者同时结束。循环播放时视频也会同步从头开始。在 RViz 中订阅 `/ft/video` 话题即可显示视频画面。

---

## 脚本配置

所有脚本顶部均有 **★ 用户配置区**，修改后重启脚本即可生效，无需改动其他代码。

### 完整版配置项（`rviz_FT_visualizer_xy.py`）

```python
# 数据文件路径（相对于脚本所在目录）
POINTS_CSV = _os.path.join(_HERE, '数据文件夹', '点云文件.csv')
TRACKS_CSV = _os.path.join(_HERE, '数据文件夹', '航迹文件.csv')

# 播放控制
PLAY_MODE  = 'auto'    # 'auto' 自动播放 / 'manual' 键盘控制
RATE_HZ    = 5.0       # 帧率（Hz）
LOOP       = True      # 是否循环

# 显示范围（米）
X_MIN      = -100.0    # 前向起始距离
X_MAX      =  300.0    # 前向最大距离
Y_RANGE    =  100.0    # 横向范围（±Y_RANGE）
Z_FILTER_MIN = -9999.0 # 高度下限（-9999 = 不过滤）
Z_FILTER_MAX =  9999.0 # 高度上限（9999 = 不过滤）

# 色带范围（-9999 = 自动取 2%/98% 百分位）
Z_MIN      = -9999.0
Z_MAX      = -9999.0

# 点云过滤
DOA_FILTER = 1         # 仅显示 DOAMethod == 1 的点（-1 = 不过滤）

# 航迹显示
SHOW_TRACKS = True     # False = 隐藏所有航迹框

# 关注目标高亮
FOCUS_ID    = -1       # 将指定 assoTrkID 的点显示为红色（-1 = 不高亮）
```

### 简化版配置项（`rviz_FT_visualizer_xy_simple.py`）

xy_simple 在完整版基础上额外包含 **坐标尺** 和 **同步视频播放** 功能，但去掉了航迹相关配置（`TRACKS_CSV`、`DOA_FILTER`、`SHOW_TRACKS`、`FOCUS_ID`）。

```python
# 坐标尺 1（发布到 /ft/ruler1）
SHOW_RULER_1     = True              # 是否显示坐标尺 1
RULER_1_AXIS     = 'x'               # 坐标尺方向：'x' 沿 X 轴 / 'y' 沿 Y 轴
RULER_1_OFFSET   = 0.0               # 坐标尺沿正交方向的偏移量（m）
RULER_1_INTERVAL = 50.0              # 相邻标记间隔（m）
RULER_1_LENGTH   = 300.0             # 坐标尺总长度（m），从坐标原点向正方向延伸
RULER_1_FONT     = 1.0               # 字体大小（scale.z）
RULER_1_COLOR    = [0.8, 0.8, 0.8]   # RGB 颜色（0~1）

# 坐标尺 2（发布到 /ft/ruler2）
SHOW_RULER_2     = False             # 是否显示坐标尺 2
RULER_2_AXIS     = 'y'               # 坐标尺方向：'x' 沿 X 轴 / 'y' 沿 Y 轴
RULER_2_OFFSET   = 0.0               # 坐标尺沿正交方向的偏移量（m）
RULER_2_INTERVAL = 50.0              # 相邻标记间隔（m）
RULER_2_LENGTH   = 200.0             # 坐标尺总长度（m），从坐标原点向正方向延伸
RULER_2_FONT     = 1.0               # 字体大小（scale.z）
RULER_2_COLOR    = [0.8, 0.8, 0.8]   # RGB 颜色（0~1）

# 同步视频（空字符串 = 不播放）
VIDEO_PATH = _os.path.join(_HERE, 'data', 'xxx.avi')
```

坐标尺通过 `/ft/ruler1` 和 `/ft/ruler2`（MarkerArray）分别发布，视频画面通过 `/ft/video`（Image）发布。

### 简化版配置项（`rviz_FT_visualizer_xz_simple.py`）

与 xy 完整版相比，去掉了航迹相关配置（`TRACKS_CSV`、`DOA_FILTER`、`SHOW_TRACKS`、`FOCUS_ID`），默认显示范围较小（`X_MAX=30`, `Y_RANGE=50`）。

### 并行启动器配置（`launch_visualizers.py`）

```python
# 要同时启动的脚本（取消注释即可加入）
SCRIPTS = [
    'rviz_FT_visualizer_xy_simple.py',
    'rviz_FT_visualizer_xz_simple.py',
    # 'rviz_FT_visualizer_xy.py',
    # 'rviz_FT_visualizer_xz.py',
]

# 启动延迟（秒），0 = 立即开始
STARTUP_DELAY = 0
```

---

## 启动方式

### 方式一：单独启动某个脚本

```bash
# 确保 ROS2 环境已 source
python rviz_FT_visualizer_xy.py
python rviz_FT_visualizer_xz.py
python rviz_FT_visualizer_xy_simple.py
python rviz_FT_visualizer_xz_simple.py
```

也可通过 `ros2 run` 传入参数覆盖配置区的默认值：

```bash
# 示例：指定文件路径、帧率、手动模式
python rviz_FT_visualizer_xy.py --ros-args \
  -p points_csv:=/path/to/points.csv \
  -p tracks_csv:=/path/to/tracks.csv \
  -p rate:=10.0 \
  -p play_mode:=manual \
  -p show_tracks:=false \
  -p focus_id:=4 \
  -p doa_filter:=-1 \
  -p z_filter_min:=-0.5 \
  -p z_filter_max:=2.0
```

### 方式二：并行启动多个脚本（推荐）

编辑 `launch_visualizers.py` 的配置区，然后运行：

```bash
python launch_visualizers.py
```

按 `Ctrl+C` 同时终止所有子进程。

### manual 模式键盘控制

在 `PLAY_MODE = 'manual'` 时，在终端输入命令后按回车：

| 按键 | 操作 |
|------|------|
| `n` 或直接回车 | 下一帧 |
| `p` | 上一帧 |
| `r` | 跳回第一帧 |
| `q` | 退出 |

---

## RViz 配置

### 通用设置

1. 打开 RViz2：`rviz2`
2. 左侧 **Global Options** → **Fixed Frame** 设为 `radar`

> 各脚本启动时会自动通过 `StaticTransformBroadcaster` 发布 `radar → map` 的静态 TF（identity，位移为零），避免 RViz 出现 "Could not transform from [radar] to [map]" 报错。如果雷达有实际安装偏移，可编辑脚本中 `TransformStamped` 的 `transform.translation` 字段。

### 订阅话题

| 话题 | 消息类型 | 说明 | 适用脚本 |
|------|----------|------|----------|
| `/ft/points_xy` | PointCloud2 | 俯视点云 | xy、xy_simple |
| `/ft/points_xz` | PointCloud2 | 侧视点云 | xz、xz_simple |
| `/ft/track_boxes` | MarkerArray | 航迹框 + ID 标签 | xy |
| `/ft/colorbar` | Image | 高度色带 | xy、xy_simple |
| `/ft/colorbar_xz` | Image | 高度色带（侧视） | xz、xz_simple |
| `/ft/frame_info` | MarkerArray | 当前帧 ID 文字 | 全部 |
| `/ft/ruler1` | MarkerArray | 坐标尺 1 数字标记 | xy_simple |
| `/ft/ruler2` | MarkerArray | 坐标尺 2 数字标记 | xy_simple |
| `/ft/video` | Image | 同步视频画面 | xy_simple |

### 添加显示项步骤

1. 点击左下角 **Add**
2. 选择 **By topic**，找到对应话题，点击 **OK**
3. PointCloud2 显示项：将 **Color Transformer** 改为 **RGB8**

### XZ 侧视角设置

在 RViz 的 Views 面板中：
- **Type** 选 `Orthographic`
- 将视角旋转至从 Y 轴方向观察（俯仰角 0°，偏航角 90°）

---

## 参数速查

所有 ROS2 参数均可在启动时通过 `--ros-args -p 参数名:=值` 覆盖。

### 通用参数（所有脚本）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `points_csv` | str | 配置区路径 | 点云文件路径 |
| `fixed_frame` | str | `radar` | RViz 坐标系名称 |
| `rate` | float | `5.0` | 播放帧率（Hz） |
| `loop` | bool | `true` | 是否循环播放 |
| `play_mode` | str | `auto` | `auto` 或 `manual` |
| `x_min` | float | 见配置区 | 前向显示起始距离（m） |
| `x_max` | float | 见配置区 | 前向显示最大距离（m） |
| `y_range` | float | 见配置区 | 横向显示范围（m） |
| `z_filter_min` | float | `-9999.0` | 高度下限过滤（m），-9999 = 不过滤 |
| `z_filter_max` | float | `9999.0` | 高度上限过滤（m），9999 = 不过滤 |
| `z_min` | float | `-9999.0` | 色带下限（-9999 = 自动） |
| `z_max` | float | `-9999.0` | 色带上限（-9999 = 自动） |

### xy 完整版专属

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `tracks_csv` | str | 配置区路径 | 航迹文件路径 |
| `doa_filter` | int | `1` | DOAMethod 过滤值（-1 = 不过滤） |
| `show_tracks` | bool | `true` | 是否显示航迹框 |
| `focus_id` | int | `-1` | 高亮目标 assoTrkID（-1 = 不高亮） |

### xy_simple 专属

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `show_ruler_1` | bool | `true` | 是否显示坐标尺 1 |
| `ruler_1_axis` | str | `x` | 坐标尺 1 方向（`x` / `y`） |
| `ruler_1_offset` | float | `0.0` | 坐标尺 1 正交偏移（m） |
| `ruler_1_interval` | float | `50.0` | 坐标尺 1 标记间隔（m） |
| `ruler_1_length` | float | `300.0` | 坐标尺 1 总长度（m） |
| `ruler_1_font` | float | `1.0` | 坐标尺 1 字体大小 |
| `ruler_1_color` | float[] | `[0.8,0.8,0.8]` | 坐标尺 1 RGB 颜色 |
| `show_ruler_2` | bool | `false` | 是否显示坐标尺 2 |
| `ruler_2_axis` | str | `y` | 坐标尺 2 方向（`x` / `y`） |
| `ruler_2_offset` | float | `0.0` | 坐标尺 2 正交偏移（m） |
| `ruler_2_interval` | float | `50.0` | 坐标尺 2 标记间隔（m） |
| `ruler_2_length` | float | `200.0` | 坐标尺 2 总长度（m） |
| `ruler_2_font` | float | `1.0` | 坐标尺 2 字体大小 |
| `ruler_2_color` | float[] | `[0.8,0.8,0.8]` | 坐标尺 2 RGB 颜色 |
| `video_path` | str | `''` | 同步视频文件路径（空字符串 = 不播放） |

### xz 完整版专属

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `doa_filter` | int | `1` | DOAMethod 过滤值（-1 = 不过滤） |
| `focus_id` | int | `-1` | 高亮目标 assoTrkID（-1 = 不高亮） |
