# FT 雷达数据 RViz 可视化工具

基于 ROS2 + RViz 的雷达点云与航迹数据逐帧回放工具，支持俯视（XY）和侧视（XZ）两种视角，提供完整版（含航迹）和简化版（纯点云）两套脚本。

---

## 目录

- [文件说明](#文件说明)
- [环境准备](#环境准备)
- [数据准备](#数据准备)
- [各脚本配置项](#各脚本配置项)
- [启动方式](#启动方式)
- [RViz 配置](#rviz-配置)
- [参数速查](#参数速查)

---

## 文件说明

| 文件 | 用途 | 所需数据 | 特色功能 |
|------|------|----------|----------|
| `rviz_FT_visualizer_xy.py` | 完整版·俯视图（XY） | 点云 CSV + 航迹 Trks CSV | 航迹框、DOA 过滤、目标高亮 |
| `rviz_FT_visualizer_xz.py` | 完整版·侧视图（XZ） | 点云 CSV | DOA 过滤、目标高亮 |
| `rviz_FT_visualizer_xy_simple.py` | 简化版·俯视图（XY） | 简化点云 CSV（4列） | **坐标尺**、**视频同步播放**、Z 轴过滤 |
| `rviz_FT_visualizer_xz_simple.py` | 简化版·侧视图（XZ） | 简化点云 CSV（4列） | Z 轴过滤 |
| `launch_visualizers.py` | 并行启动器 | — | 多脚本同时启动、启动延迟 |
| `requirements.txt` | Python 依赖列表 | — | — |

---

## 环境准备

### 1. 安装 ROS2

依赖 ROS2（推荐 Humble 或 Iron），安装参考：https://docs.ros.org/en/humble/Installation.html

每次打开终端需 source 环境：

```bash
# Linux
source /opt/ros/humble/setup.bash

# Windows（PowerShell）
C:\dev\ros2_humble\local_setup.ps1
```

### 2. Python 虚拟环境（可选）

```bash
python -m venv .venv

# 激活（Windows PowerShell）
.venv\Scripts\Activate.ps1
```

### 3. 安装依赖

```bash
pip install -r requirements.txt
```

| 包 | 最低版本 | 用途 |
|----|----------|------|
| numpy | 1.21.0 | 点云数组运算 |
| pandas | 1.3.0 | CSV 读取 |
| opencv-python | 4.5.0 | 色带生成、视频解码 |

> ROS2 包（`rclpy`、`sensor_msgs`、`visualization_msgs`、`cv_bridge`、`tf2_ros`）随 ROS2 安装。

---

## 数据准备

### 完整版点云 CSV（xy / xz）

必须包含以下列（列名需完全匹配）：

| 列名 | 类型 | 说明 |
|------|------|------|
| `frameID` | int | 帧编号 |
| `xpos` | float | 前向距离（m） |
| `ypos` | float | 横向距离（m） |
| `zpos` | float | 高度（m） |
| `assoTrkID` | int | 关联航迹 ID |
| `DOAMethod` | int | DOA 方法标志（1 = 有效） |

### 简化版点云 CSV（xy_simple / xz_simple）

仅需 4 列，**按列位置读取，列名不限**：

| 列位置 | 字段 | 说明 |
|--------|------|------|
| A（第1列） | frameID | 帧编号 |
| B（第2列） | xpos | 前向距离（m） |
| C（第3列） | ypos | 横向距离（m） |
| D（第4列） | zpos | 高度（m） |

### 航迹 CSV（仅 xy 完整版需要）

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

### 同步视频文件（仅 xy_simple 支持）

格式：`.avi`（其他格式可修改 `cv2.VideoCapture` 自动识别）。

视频将与点云同步播放：总播放时间由 CSV 总帧数 ÷ 播放帧率确定，视频帧率自动计算以保证两者同时结束。

---

## 各脚本配置项

所有脚本顶部均有 **★ 用户配置区**，修改后重启即可生效。

### xy_simple（🌟 功能最全）

```python
# 数据文件路径（相对于脚本所在目录）
POINTS_CSV = _os.path.join(_HERE, 'data', 'xxx.csv')

# 播放控制
PLAY_MODE  = 'auto'        # 'auto' / 'manual'
RATE_HZ    = 5.0           # 帧率（Hz）
LOOP       = True          # 是否循环

# 显示范围（m）
X_MIN      = -100.0        # 前向起始
X_MAX      =  300.0        # 前向最远
Y_RANGE    =  100.0        # 横向范围（±）
Z_FILTER_MIN = -1.0        # Z 下限过滤（-9999=不过滤）
Z_FILTER_MAX =  9999.0     # Z 上限过滤（9999=不过滤）

# 色带范围（-9999=自动取 2%/98% 百分位）
Z_MIN      = -9999.0
Z_MAX      = -9999.0

# 坐标尺 1（发布到 /ft/ruler1）
SHOW_RULER_1     = True              # 是否显示
RULER_1_AXIS     = 'x'               # 'x' 沿 X 轴 / 'y' 沿 Y 轴
RULER_1_OFFSET   = 0.0               # 正交方向偏移（m）
RULER_1_INTERVAL = 50.0              # 相邻标记间隔（m）
RULER_1_LENGTH   = 300.0             # 总长度（m），从原点向正方向延伸
RULER_1_FONT     = 1.0               # 字体大小
RULER_1_COLOR    = [0.8, 0.8, 0.8]   # RGB 颜色（0~1）

# 坐标尺 2（发布到 /ft/ruler2）
SHOW_RULER_2     = False             # 是否显示
RULER_2_AXIS     = 'y'               # 'x' 沿 X 轴 / 'y' 沿 Y 轴
RULER_2_OFFSET   = 0.0               # 正交方向偏移（m）
RULER_2_INTERVAL = 50.0              # 相邻标记间隔（m）
RULER_2_LENGTH   = 200.0             # 总长度（m），从原点向正方向延伸
RULER_2_FONT     = 1.0               # 字体大小
RULER_2_COLOR    = [0.8, 0.8, 0.8]   # RGB 颜色（0~1）

# 同步视频（空字符串 = 不播放）
VIDEO_PATH = _os.path.join(_HERE, 'data', 'xxx.avi')
```

### xy（完整版）

在以上基础上增加：

```python
TRACKS_CSV = _os.path.join(_HERE, 'data', 'xxxTrks.csv')  # 航迹文件
DOA_FILTER = 1          # DOAMethod 过滤（-1=不过滤）
SHOW_TRACKS = True      # 航迹框开关
FOCUS_ID    = -1        # 高亮目标 assoTrkID（-1=不高亮）
```

### xz / xz_simple

默认范围较小（`X_MAX=30`, `Y_RANGE=50`），无航迹和视频功能。

### 并行启动器（launch_visualizers.py）

```python
SCRIPTS = [
    'rviz_FT_visualizer_xy_simple.py',
    'rviz_FT_visualizer_xz_simple.py',
]
STARTUP_DELAY = 0   # 启动延迟（秒），0=立即
```

---

## 启动方式

### 方式一：单独启动

```bash
python rviz_FT_visualizer_xy_simple.py
```

通过 `--ros-args -p` 覆盖配置区默认值：

```bash
python rviz_FT_visualizer_xy_simple.py --ros-args \
  -p rate:=10.0 \
  -p loop:=false \
  -p z_filter_min:=-0.5 \
  -p ruler_axis:='y' \
  -p video_path:=''
```

### 方式二：并行启动（推荐）

编辑 `launch_visualizers.py` 的 `SCRIPTS` 列表，然后：

```bash
python launch_visualizers.py
```

按 `Ctrl+C` 同时终止所有子进程。

### manual 模式键盘控制

`PLAY_MODE = 'manual'` 时，终端输入后回车：

| 按键 | 操作 |
|------|------|
| `n` 或直接回车 | 下一帧 |
| `p` | 上一帧 |
| `r` | 跳回第一帧 |
| `q` | 退出 |

---

## RViz 配置

### 通用设置

1. 启动 RViz2：`rviz2`
2. **Global Options** → **Fixed Frame** 设为 `radar`

> 每个脚本启动时会自动发布 `radar → map` 的静态 TF（identity），避免 "Could not transform" 错误。如雷达有实际安装偏移，可在脚本的 TF 发布段修改 `TransformStamped` 的 `transform.translation`。

### XY 视图的话题订阅

| 话题 | 消息类型 | 说明 | 脚本 |
|------|----------|------|------|
| `/ft/points_xy` | PointCloud2 | 俯视点云（按高度着色） | xy, xy_simple |
| `/ft/track_boxes` | MarkerArray | 航迹框 + ID 标签 | xy |
| `/ft/colorbar` | Image | 高度色带 | xy, xy_simple |
| `/ft/frame_info` | MarkerArray | 当前帧 ID 文字 | 全部 |
| `/ft/ruler` | MarkerArray | 坐标尺数字标记 | xy_simple |
| `/ft/video` | Image | 同步视频画面 | xy_simple |

### XZ 视图的话题订阅

| 话题 | 消息类型 | 说明 | 脚本 |
|------|----------|------|------|
| `/ft/points_xz` | PointCloud2 | 侧视点云 | xz, xz_simple |
| `/ft/colorbar_xz` | Image | 高度色带 | xz, xz_simple |
| `/ft/frame_info` | MarkerArray | 当前帧 ID | 全部 |

### 添加显示项

1. 点击左下角 **Add**
2. 选 **By topic**，找到对应话题，点 **OK**
3. PointCloud2 需将 **Color Transformer** 设为 **RGB8**

### XZ 侧视角设置

Views 面板中：**Type** → `Orthographic`，将视角旋转至从 Y 轴方向观察。

---

## 参数速查

启动时通过 `--ros-args -p 参数:=值` 覆盖。

### 通用参数（所有脚本）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `points_csv` | str | 见配置区 | 点云文件路径 |
| `fixed_frame` | str | `radar` | RViz 坐标系 |
| `rate` | float | `5.0` | 播放帧率（Hz） |
| `loop` | bool | `true` | 是否循环 |
| `play_mode` | str | `auto` | `auto` / `manual` |
| `x_min` | float | 见配置区 | 前向起始距离（m） |
| `x_max` | float | 见配置区 | 前向最大距离（m） |
| `y_range` | float | 见配置区 | 横向范围（±，m） |
| `z_filter_min` | float | `-9999.0` | Z 下限过滤（-9999=不过滤） |
| `z_filter_max` | float | `9999.0` | Z 上限过滤（9999=不过滤） |
| `z_min` | float | `-9999.0` | 色带下限（-9999=自动） |
| `z_max` | float | `-9999.0` | 色带上限（-9999=自动） |

### xy 完整版专属

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `tracks_csv` | str | 见配置区 | 航迹文件路径 |
| `doa_filter` | int | `1` | DOAMethod 过滤（-1=不过滤） |
| `show_tracks` | bool | `true` | 航迹框开关 |
| `focus_id` | int | `-1` | 高亮目标 ID（-1=不高亮） |

### xz 完整版专属

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `doa_filter` | int | `1` | DOAMethod 过滤（-1=不过滤） |
| `focus_id` | int | `-1` | 高亮目标 ID（-1=不高亮） |

### xy_simple 专属

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `show_ruler` | bool | `true` | 坐标尺开关 |
| `ruler_axis` | str | `x` | 坐标尺方向（`x` / `y`） |
| `ruler_offset` | float | `0.0` | 坐标尺正交偏移（m） |
| `ruler_interval` | float | `50.0` | 标记间隔（m） |
| `ruler_length` | float | `300.0` | 总长度（m） |
| `video_path` | str | `''` | 视频文件路径（空=不播放） |
