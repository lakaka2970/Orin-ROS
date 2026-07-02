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
#   bash scripts/start.sh --no-adc        # 不启动 adc_rx, logging 不录 adc.bin
#
#   开发管线 (调试/验证雷达硬件, 与 ROS 采集互斥 — 共享 /dev/video0):
#   bash scripts/start.sh --capture-only           # 仅采集雷达原始数据
#   bash scripts/start.sh --capture-only --rsps    # 采集 + RSPS 离线点云可视化
#   bash scripts/start.sh --capture --rsps         # 采集 + RSPS → 自动启动 ROS 框架
#
# 可选参数 (任意顺序):
#   --analog       使用模拟 ADC 数据源 (噪声池/.bin), 默认 real (硬件设备)
#   --no-adc       不启动 adc_rx 节点, logging_node 自动关闭 adc.bin 录制
#   --py-rx        使用 Python 版 rx 节点 (默认 C++)
#   --rviz         同时启动 RViz2
#
#   开发管线 (调试/验证雷达硬件, 与 ROS 框架互斥 — 共享 /dev/video0):
#   --capture      启动 ROS 框架前先采集雷达原始数据 + 可选 RSPS 处理
#   --capture-only 仅采集雷达数据 + 可选 RSPS 处理 (不启动 ROS 框架)
#   --rsps         采集后运行 RSPS 点云可视化处理 (需配合 --capture 或 --capture-only)
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
NO_ADC=false
DO_CAPTURE=false
DO_RSPS=false
CAPTURE_ONLY=false

for arg in "$@"; do
    case "$arg" in
        python|cuda|both|both_compare) RSP_MODE="$arg" ;;
        --analog) ADC_SOURCE="analog" ;;
        --py-rx)  USE_PY_RX=true ;;
        --rviz)   USE_RVIZ=true ;;
        --no-adc) NO_ADC=true ;;
        --capture) DO_CAPTURE=true ;;
        --capture-only) DO_CAPTURE=true; CAPTURE_ONLY=true ;;
        --rsps)   DO_RSPS=true ;;
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

# ── 3. RMW 检查 ──
echo "[ft] RMW 实现: ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (默认 FastDDS, 内置 SHM)}"

# ── 3.5 重启 ROS2 daemon (防止上次异常退出导致 DDS 发现失败 → 节点间通信断开) ──
# 症状: ros2 node list 能看到节点但 topic 无数据, logging 收到 0 条消息
_ft_daemon_pid=$(pgrep -f "_ros2_daemon" 2>/dev/null || true)
if [ -n "$_ft_daemon_pid" ]; then
    ros2 daemon stop 2>/dev/null || true
    # 等待旧 daemon 完全退出
    for _ in $(seq 1 10); do
        kill -0 "$_ft_daemon_pid" 2>/dev/null || break
        sleep 0.3
    done
fi
ros2 daemon start 2>/dev/null || true
echo "[ft] ROS2 daemon 已重启"

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

# ══════════════════════════════════════════════════════════════════════════════
# 4.6 开发管线: 雷达原始数据采集 + RSPS 离线点云处理 (可选)
#
# 与 ROS 生产管线互斥 — 都需独占 /dev/video0 (/dev/radar_ctrx0)。
# 采集在 ROS 框架启动前完成 (20 帧 × 2 设备 ≈ 2-3 秒)。
# ══════════════════════════════════════════════════════════════════════════════
CAPTURE_SCRIPT="$PROJECT_ROOT/src/integration-carkit88c0-gmsl/scripts/capture_video0_2048x1024.sh"
CAPTURE_DIR="$PROJECT_ROOT/src/integration-carkit88c0-gmsl/scripts"
RSPS_SCRIPT="$CAPTURE_DIR/RSPS/RSPS_main.py"
RSPS_OUTPUT_DIR="$CAPTURE_DIR/output/rsps_plots"

if $DO_CAPTURE; then
    echo "=============================================="
    echo "  [开发管线] 雷达原始数据采集 (v4l2-ctl)"
    echo "  设备: /dev/radar_ctrx0 + /dev/radar_ctrx1"
    echo "  输出: $CAPTURE_DIR/output/"
    echo "=============================================="

    if [ -f "$CAPTURE_SCRIPT" ]; then
        echo "[ft] 启动雷达数据采集..."
        cd "$CAPTURE_DIR"
        set +e
        bash "$CAPTURE_SCRIPT"
        CAPTURE_EXIT=$?
        set -e
        if [ $CAPTURE_EXIT -eq 0 ]; then
            echo "[ft] [OK] 雷达数据采集完成"
            echo "[ft]   ctrx0_raw.bin → $CAPTURE_DIR/output/"
            echo "[ft]   ctrx1_raw.bin → $CAPTURE_DIR/output/"
        else
            echo "[ft] [WARN] 雷达数据采集异常"
        fi
        cd "$PROJECT_ROOT"
    else
        echo "[ft] [WARN] 采集脚本不存在: $CAPTURE_SCRIPT"
    fi

    # ── 4.7 RSPS 离线点云可视化 (可选) ──
    if $DO_RSPS; then
        echo ""
        echo "=============================================="
        echo "  [开发管线] RSPS 雷达点云离线处理"
        echo "  输入: $CAPTURE_DIR/output/ctrx0_raw.bin"
        echo "=============================================="

        if [ -f "$RSPS_SCRIPT" ]; then
            echo "[ft] 运行 RSPS 点云处理 (Range/Doppler FFT + CFAR + DOA)..."
            cd "$CAPTURE_DIR"
            # 使用 Agg 后端避免无显示器报错
            set +e
            MPLBACKEND=Agg python3 "$RSPS_SCRIPT" \
                --save --save-dir "$RSPS_OUTPUT_DIR"
            RSPS_EXIT=$?
            set -e
            if [ $RSPS_EXIT -eq 0 ]; then
                echo "[ft] [OK] RSPS 点云处理完成"
                echo "[ft]   可视化输出: $RSPS_OUTPUT_DIR/"
                if [ -d "$RSPS_OUTPUT_DIR" ]; then
                    ls -lh "$RSPS_OUTPUT_DIR/" 2>/dev/null
                fi
            else
                echo "[ft] [WARN] RSPS 点云处理异常"
            fi
            cd "$PROJECT_ROOT"
        else
            echo "[ft] [WARN] RSPS_main.py 不存在: $RSPS_SCRIPT"
        fi
    fi

    echo ""

    # --capture-only: 仅开发管线, 不启动 ROS 框架
    if $CAPTURE_ONLY; then
        echo "[ft] [开发管线] 采集完成, 退出 (--capture-only)"
        echo "[ft]   提示: 去掉 --capture-only 可在采集后自动启动 ROS 框架"
        exit 0
    fi
fi

# ══════════════════════════════════════════════════════════════════════════════
# 5. 生产管线: ROS2 框架启动
#    adc_rx (C++) → /dev/video0 → ROS2 topics → RSP → det_list
# ══════════════════════════════════════════════════════════════════════════════
echo "=============================================="
echo "  FT Radar Framework 启动"
echo "  RSP 模式:   $RSP_MODE"
echo "  Rx 实现:    $RX_IMPL"
echo "  ADC 数据源: $ADC_SOURCE"
if $NO_ADC; then
    echo "  ADC 节点:   禁用 (--no-adc)"
fi
echo "  RMW:        ${RMW_IMPLEMENTATION:-FastDDS (default)}"
echo "=============================================="

# 构建 launch 参数
_ft_launch_args=(
    "rsp_mode:=$RSP_MODE"
    "rx_impl:=$RX_IMPL"
    "adc_source:=$ADC_SOURCE"
    "no_adc:=$NO_ADC"
)
# --no-adc 时自动关闭 logging 的 ADC 录制 (无 adc_rx → 无数据可录)
if $NO_ADC; then
    _ft_launch_args+=("log_adc:=false")
fi

ros2 launch ft_framework ft_radar_launch.py "${_ft_launch_args[@]}" &

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
