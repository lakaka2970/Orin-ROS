#!/bin/bash
# ============================================================================
# FT Radar Framework — 一键启动脚本
# ============================================================================
# 封装环境加载 + 构建检查 + ros2 launch, 一行命令启动整个框架.
#
# 用法:
#   bash scripts/start.sh                 # 默认 cuda 模式 + C++ rx 节点
#   bash scripts/start.sh python          # Python RSP 模式
#   bash scripts/start.sh cuda            # CUDA RSP 模式
#   bash scripts/start.sh both_compare    # 双路对比模式
#
# 可选参数 (任意顺序):
#   --py-rx    使用 Python 版 rx 节点 (默认 C++)
#   --rviz     同时启动 RViz2
#
# 作者: zhengyuan.liu
# 日期: 2026-06-18
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

RSP_MODE="cuda"
USE_PY_RX=false
USE_RVIZ=false

for arg in "$@"; do
    case "$arg" in
        python|cuda|both|both_compare) RSP_MODE="$arg" ;;
        --py-rx) USE_PY_RX=true ;;
        --rviz)  USE_RVIZ=true ;;
        *)       echo "未知参数: $arg"; exit 1 ;;
    esac
done

# ── 1. 加载环境 ──
cd "$PROJECT_ROOT"
source "$SCRIPT_DIR/env.sh"

# ── 2. 检查构建 ──
if [ ! -f "$PROJECT_ROOT/install/setup.bash" ]; then
    echo "[ft] 工作空间未构建，正在自动构建..."
    bash "$SCRIPT_DIR/build.sh"
    source "$PROJECT_ROOT/install/setup.bash"
fi

# ── 3. 检查 CycloneDDS ──
if [ "$RMW_IMPLEMENTATION" != "rmw_cyclonedds_cpp" ]; then
    echo "[ft] [WARN] CycloneDDS 未安装, 大消息可能丢包。安装:"
    echo "    bash scripts/install_deps.sh --with-cyclonedds"
    echo ""
fi

# ── 4. 确定 rx 实现 ──
if $USE_PY_RX; then
    RX_IMPL="python"
else
    RX_IMPL="cpp"
fi

# ── 5. 启动 ──
echo "=============================================="
echo "  FT Radar Framework 启动"
echo "  RSP 模式: $RSP_MODE"
echo "  Rx 实现:  $RX_IMPL"
echo "  RMW:      ${RMW_IMPLEMENTATION:-FastDDS (default)}"
echo "=============================================="

ros2 launch ft_framework ft_radar_launch.py \
    rsp_mode:=$RSP_MODE \
    rx_impl:=$RX_IMPL &

LAUNCH_PID=$!

# ── 6. 可选 RViz ──
if $USE_RVIZ; then
    sleep 2
    if [ -f "$PROJECT_ROOT/config/ft_radar.rviz" ]; then
        rviz2 -d "$PROJECT_ROOT/config/ft_radar.rviz" &
    else
        rviz2 &
    fi
fi

echo ""
echo "  按 Ctrl+C 停止所有节点"
echo ""

cleanup() {
    echo ""
    echo "[ft] 正在停止..."
    kill $LAUNCH_PID 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $LAUNCH_PID
