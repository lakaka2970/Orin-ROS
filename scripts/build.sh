#!/bin/bash
# ============================================================================
# FT Radar Framework — 一步式环境加载与构建脚本
# ============================================================================
# 用法:
#   bash scripts/build.sh                          # 增量构建
#   bash scripts/build.sh --clean                  # 清理后重新构建
#   bash scripts/build.sh --launch                 # 构建后直接启动
#   bash scripts/build.sh --clean --launch         # 清理→构建→启动
#
# 特性:
#   - 自动加载 ROS2 Humble 环境
#   - 一步完成两个包的按序构建
#   - 输出启动所需的 exact 命令
#
# 注意:
#   脚本内的 source 仅影响脚本自身。构建完成后，
#   必须在当前终端手动执行以下命令加载工作空间:
#     source install/setup.bash
#
# 作者: zhengyuan.liu
# 日期: 2026.6.8
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# ===== 解析参数 =====
DO_CLEAN=false
DO_LAUNCH=false

for arg in "$@"; do
    case "$arg" in
        --clean) DO_CLEAN=true ;;
        --launch) DO_LAUNCH=true ;;
    esac
done

echo "=============================================="
echo "  FT Radar Framework — 一步式构建"
echo "  项目根目录: $PROJECT_ROOT"
echo "=============================================="

# ===== 步骤 1: 加载 ROS2 环境 =====
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[1/4] ✅ ROS2 Humble 环境已加载"
else
    echo "[ERROR] 未找到 /opt/ros/humble/setup.bash"
    echo "        请确认 ROS2 Humble 已安装"
    exit 1
fi

cd "$PROJECT_ROOT"

# ===== 步骤 2: 可选清理 =====
if $DO_CLEAN; then
    echo "[2/4] 🧹 清理旧的构建产物..."
    rm -rf build/ install/ log/
    echo "[2/4] ✅ 清理完成"
else
    echo "[2/4] ⏭️  跳过清理（增量构建）"
fi

# ===== 步骤 3: 构建 =====
# 先构建消息包，加载后构建节点包
echo "[3/4] 🔨 构建 ft_radar_msgs (自定义消息)..."
colcon build --packages-select ft_radar_msgs --symlink-install \
    --allow-overriding ft_radar_msgs \
    2>&1 | grep -v "WARNING.*doesn't exist\|WARNING.*colcon\|If you understand\|--allow-overriding\|Some selected\|This may be\|If a package\|Failure to"
if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo ""
    echo "[ERROR] ft_radar_msgs 构建失败！"
    echo "如果是因为目录重命名导致的 CMake 缓存问题，请执行:"
    echo "  bash scripts/build.sh --clean"
    exit 1
fi

# 加载消息包环境，使下游包能找到 .msg 定义
source "$PROJECT_ROOT/install/setup.bash" 2>/dev/null || true

echo "[3/4] 🔨 构建 ft_framework (10 个节点)..."
colcon build --packages-select ft_framework --symlink-install \
    2>&1 | grep -v "WARNING.*doesn't exist\|WARNING.*colcon"
if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo ""
    echo "[ERROR] ft_framework 构建失败！"
    exit 1
fi

# ===== 步骤 4: 输出结果 =====
echo ""
echo "=============================================="
echo "  ✅ 构建完成！"
echo "=============================================="
echo ""
echo "  ⚠️  构建脚本内的环境加载仅限脚本自身，"
echo "     请在当前终端执行以下命令:"
echo ""
echo "    source $PROJECT_ROOT/install/setup.bash"
echo ""
echo "  然后启动框架:"
echo ""
echo "    ros2 launch ft_framework ft_radar_launch.py"
echo ""
echo "  其他启动模式:"
echo ""
echo "    ros2 launch ft_framework ft_radar_launch.py rsp_mode:=python"
echo "    ros2 launch ft_framework ft_radar_launch.py rsp_mode:=both"
echo ""
echo "  RViz 可视化:"
echo ""
echo "    rviz2 -d $PROJECT_ROOT/config/ft_radar.rviz"
echo ""
echo "=============================================="

# ===== 可选启动（自动 source 后启动） =====
if $DO_LAUNCH; then
    echo ""
    echo "🚀 自动加载环境并启动框架..."
    source "$PROJECT_ROOT/install/setup.bash"
    ros2 launch ft_framework ft_radar_launch.py
fi
