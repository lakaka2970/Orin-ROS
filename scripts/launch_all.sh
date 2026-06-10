#!/bin/bash
# ============================================================================
# FT Radar Framework — 一键启动脚本
# ============================================================================
# 自动检测 ROS2 发行版 (Foxy / Humble) 并加载对应环境。
#
# 用法:
#   bash scripts/launch_all.sh                      # 默认 CUDA 模式
#   bash scripts/launch_all.sh python               # Python 模式
#   bash scripts/launch_all.sh cuda --rviz          # CUDA 模式 + RViz
#   bash scripts/launch_all.sh both_compare         # 双路对比模式
#
# 作者: zhengyuan.liu
# 日期: 2026.6.10
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

RSP_MODE="${1:-cuda}"
RVIZ_FLAG="$2"

# ── 自动检测 ROS2 ──
_detect_ros2() {
    for d in humble foxy; do
        [ -f "/opt/ros/$d/setup.bash" ] && echo "$d" && return
    done
    case "$(lsb_release -rs 2>/dev/null)" in
        20.04) echo "foxy" ;;
        22.04) echo "humble" ;;
        *)     echo "humble" ;;
    esac
}

ROS2_DISTRO=$(_detect_ros2)

echo "=============================================="
echo "  FT Radar Framework — 启动脚本"
echo "  ROS2:     $ROS2_DISTRO"
echo "  模式:     $RSP_MODE"
echo "=============================================="

if [ -f "/opt/ros/$ROS2_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS2_DISTRO/setup.bash"
    echo "[INFO] ROS2 $ROS2_DISTRO 环境已加载"
else
    echo "[WARN] 未找到 ROS2 环境，尝试继续..."
fi

# 加载工作空间
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
else
    echo "[WARN] 未找到 install/setup.bash，请先运行 scripts/build.sh"
fi

cd "$PROJECT_ROOT"

# 启动所有节点
echo "[INFO] 启动 FT Radar Framework (rsp_mode=$RSP_MODE)..."
ros2 launch ft_framework ft_radar_launch.py rsp_mode:=$RSP_MODE &
LAUNCH_PID=$!

# 可选启动 RViz2
if [ "$RVIZ_FLAG" = "--rviz" ]; then
    echo "[INFO] 启动 RViz2..."
    sleep 2
    if [ -f config/ft_radar.rviz ]; then
        rviz2 -d config/ft_radar.rviz &
    else
        rviz2 &
    fi
    RVIZ_PID=$!
fi

echo ""
echo "  按 Ctrl+C 停止所有节点"

cleanup() {
    echo ""
    echo "[INFO] 正在停止..."
    kill $LAUNCH_PID 2>/dev/null || true
    [ -n "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM
wait $LAUNCH_PID
