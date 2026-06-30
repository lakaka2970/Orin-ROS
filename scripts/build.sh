#!/bin/bash
# ============================================================================
# FT Radar Framework — 一步式环境加载与构建脚本
# ============================================================================
# 自动检测 ROS2 发行版:
#   Ubuntu 20.04 → Foxy   (/opt/ros/foxy/setup.bash)
#   Ubuntu 22.04 → Humble (/opt/ros/humble/setup.bash)
#
# 用法:
#   bash scripts/build.sh                          # 增量构建
#   bash scripts/build.sh --clean                  # 清理后重新构建
#   bash scripts/build.sh --launch                 # 构建后直接启动
#   bash scripts/build.sh --clean --launch         # 清理→构建→启动
#
# 作者: zhengyuan.liu
# 日期: 2026.6.10
# ============================================================================

set -e
set -o pipefail

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

# ===== 自动检测 ROS2 发行版 =====
_detect_ros2() {
    for d in humble foxy; do
        if [ -f "/opt/ros/$d/setup.bash" ]; then
            echo "$d"; return
        fi
    done
    # 回退: 从 Ubuntu 版本推断
    case "$(lsb_release -rs 2>/dev/null)" in
        20.04) echo "foxy" ;;
        22.04) echo "humble" ;;
        *)     echo "" ;;
    esac
}

ROS2_DISTRO=$(_detect_ros2)

if [ -z "$ROS2_DISTRO" ]; then
    echo "[ERROR] 无法检测 ROS2 安装。请先运行:"
    echo "          bash scripts/install_deps.sh --with-ros2"
    exit 1
fi

ROS2_SETUP="/opt/ros/$ROS2_DISTRO/setup.bash"

echo "=============================================="
echo "  FT Radar Framework — 一步式构建"
echo "  项目根目录: $PROJECT_ROOT"
echo "  ROS2:     $ROS2_DISTRO"
echo "=============================================="

# ===== 步骤 1: 加载 ROS2 环境 =====
if [ -f "$ROS2_SETUP" ]; then
    source "$ROS2_SETUP"
    echo "[1/5] [OK] ROS2 $ROS2_DISTRO 环境已加载"
else
    echo "[ERROR] 未找到 $ROS2_SETUP"
    exit 1
fi

cd "$PROJECT_ROOT"

# ===== 步骤 2: 可选清理 =====
if $DO_CLEAN; then
    echo "[2/5] [CLEAN] 清理旧的构建产物..."
    rm -rf build/ install/ log/
    echo "[2/5] [OK] 清理完成"
else
    echo "[2/5] [SKIP]  跳过清理（增量构建）"
fi

# ===== 步骤 3: 构建 =====
# 先构建消息包，加载后构建节点包
echo "[3/5] [BUILD] 构建 ft_radar_msgs (自定义消息)..."
colcon build --packages-select ft_radar_msgs --symlink-install \
    --cmake-force-configure \
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
if ! source "$PROJECT_ROOT/install/setup.bash" 2>/dev/null; then
    echo "[WARN] 无法加载 install/setup.bash, 后续包可能找不到 ft_radar_msgs"
fi

echo "[4/5] [BUILD] 构建 ft_framework (10 个节点)..."
colcon build --packages-select ft_framework --symlink-install \
    2>&1 | grep -v "WARNING.*doesn't exist\|WARNING.*colcon"
if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo ""
    echo "[ERROR] ft_framework 构建失败！"
    exit 1
fi

echo "[5/5] [BUILD] 构建 ft_rx_cpp (C++ rx 节点)..."
colcon build --packages-select ft_rx_cpp --symlink-install \
    2>&1 | grep -v "WARNING.*doesn't exist\|WARNING.*colcon"
if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo ""
    echo "[ERROR] ft_rx_cpp 构建失败！"
    exit 1
fi

# ===== 步骤 4: 输出结果 =====
echo ""
echo "=============================================="
echo "  [OK] 构建完成！"
echo "=============================================="
echo ""
echo "  一键启动 (推荐):"
echo ""
echo "    bash scripts/start.sh              # 默认 cuda + C++ rx"
echo "    bash scripts/start.sh python       # Python RSP + C++ rx"
echo "    bash scripts/start.sh cuda --rviz  # CUDA + RViz"
echo ""
echo "  或手动启动:"
echo ""
echo "    source scripts/env.sh"
echo "    ros2 launch ft_framework ft_radar_launch.py rsp_mode:=python"
echo ""
echo "=============================================="

# ===== 可选启动（自动 source 后启动） =====
if $DO_LAUNCH; then
    echo ""
    echo "[START] 自动加载环境并启动框架..."
    source "$PROJECT_ROOT/install/setup.bash"
    ros2 launch ft_framework ft_radar_launch.py
fi
