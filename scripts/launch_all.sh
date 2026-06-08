#!/bin/bash
# ============================================================================
# FT Framework — 一键启动脚本
# ============================================================================
# 用法：
#   bash scripts/launch_all.sh       # 使用 launch 文件启动全部节点
#   bash scripts/launch_all.sh --rviz # 同时启动 RViz2
#
# 作者：zhengyuan.liu
# 日期：2026.6.8
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "  FT Framework — 启动脚本"
echo "=============================================="

# 加载 ROS2 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# 加载工作空间
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
else
    echo "[WARN] 未找到 install/setup.bash，请先运行 scripts/build.sh"
fi

cd "$PROJECT_ROOT"

# 启动全部节点
echo "[INFO] 启动 FT Framework (10 个节点)..."
ros2 launch ft_framework ft_framework.launch.py &
LAUNCH_PID=$!

# 可选启动 RViz2
if [ "$1" = "--rviz" ]; then
    echo "[INFO] 等待节点初始化..."
    sleep 2
    echo "[INFO] 启动 RViz2..."
    if [ -f config/ft_framework.rviz ]; then
        rviz2 -d config/ft_framework.rviz &
        RVIZ_PID=$!
    else
        rviz2 &
        RVIZ_PID=$!
    fi
fi

echo ""
echo "=============================================="
echo "  FT Framework 已启动"
echo "  按 Ctrl+C 停止所有节点"
echo "=============================================="

# 捕获退出信号
cleanup() {
    echo ""
    echo "[INFO] 正在停止所有节点..."
    kill $LAUNCH_PID 2>/dev/null
    if [ -n "$RVIZ_PID" ]; then
        kill $RVIZ_PID 2>/dev/null
    fi
    echo "[OK] 已停止"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 等待
wait $LAUNCH_PID
