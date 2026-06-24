#!/bin/bash
# ============================================================================
# FT Radar Framework — 一键启动脚本
# ============================================================================
# 封装环境加载 + 构建检查 + ros2 launch, 一行命令启动整个框架.
#
# 用法:
#   bash scripts/start.sh                 # 默认 cuda 模式 + C++ rx 节点 (real ADC)
#   bash scripts/start.sh python          # Python RSP 模式
#   bash scripts/start.sh cuda            # CUDA RSP 模式
#   bash scripts/start.sh both_compare    # 双路对比模式
#
# 可选参数 (任意顺序):
#   --analog   使用模拟 ADC 数据源 (噪声池/.bin), 默认 real (硬件设备)
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
ADC_SOURCE="real"

for arg in "$@"; do
    case "$arg" in
        python|cuda|both|both_compare) RSP_MODE="$arg" ;;
        --analog) ADC_SOURCE="analog" ;;
        --py-rx)  USE_PY_RX=true ;;
        --rviz)   USE_RVIZ=true ;;
        *)        echo "未知参数: $arg"; exit 1 ;;
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

# ── 4.5 清理上次运行残留的进程 ──
# 多次 Ctrl+C 或异常退出可能导致 Python 节点残留, 占用节点名
_ft_kill_stale() {
    local patterns=(
        "install/ft_framework/lib/ft_framework/"
        "install/ft_rx_cpp/lib/ft_rx_cpp/"
    )
    local killed=false
    for pat in "${patterns[@]}"; do
        local pids=$(pgrep -f "$pat" 2>/dev/null || true)
        if [ -n "$pids" ]; then
            echo "[ft] 清理残留进程: $(echo $pids | wc -w) 个"
            kill $pids 2>/dev/null || true
            sleep 0.5
            # 强制终止还在的
            pids=$(pgrep -f "$pat" 2>/dev/null || true)
            [ -n "$pids" ] && kill -9 $pids 2>/dev/null || true
            killed=true
        fi
    done
    $killed && sleep 0.5 || true
}
_ft_kill_stale

# ── 5. 启动 ──
echo "=============================================="
echo "  FT Radar Framework 启动"
echo "  RSP 模式:   $RSP_MODE"
echo "  Rx 实现:    $RX_IMPL"
echo "  ADC 数据源: $ADC_SOURCE"
echo "  RMW:        ${RMW_IMPLEMENTATION:-FastDDS (default)}"
echo "=============================================="

ros2 launch ft_framework ft_radar_launch.py \
    rsp_mode:=$RSP_MODE \
    rx_impl:=$RX_IMPL \
    adc_source:=$ADC_SOURCE &

LAUNCH_PID=$!

# ── 6. 可选 RViz ──
if $USE_RVIZ; then
    sleep 2
    if [ -f "$PROJECT_ROOT/config/ft_radar.rviz" ]; then
        rviz2 -d "$PROJECT_ROOT/config/ft_radar.rviz" &
    fi
fi

echo ""
echo "  按 Ctrl+C 停止所有节点"
echo ""

cleanup() {
    echo ""
    echo "[ft] 正在停止所有节点..."
    # 先向 launch 进程组发 SIGTERM，让节点有机会优雅退出
    kill -TERM -$LAUNCH_PID 2>/dev/null || kill $LAUNCH_PID 2>/dev/null || true
    sleep 1
    # 再次确保所有 FT 进程已终止
    _ft_kill_stale
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $LAUNCH_PID
