#!/bin/bash
# ============================================================================
# FT Radar Framework — 环境加载脚本
# ============================================================================
# 一个 source 完成:
#   1. 检测并载入 ROS2 发行版环境
#   2. 强制使用 FastDDS (内置 SHM 共享内存传输) — 禁止 CycloneDDS
#   3. 载入工作空间 install/setup.bash
#
# 用法:
#   source scripts/env.sh
#
# 免 source 自动加载 (推荐):
#   echo "source ~/Orin-ROS/scripts/env.sh" >> ~/.bashrc
#
# DDS 策略 (2026-07-02 更新):
#   本框架仅支持 FastDDS (rmw_fastrtps_cpp)。
#   CycloneDDS 0.7.0 无 SHM 支持, 32MB ADC 帧走 UDP loopback
#   会被切分为 512 个 RTPS 分片, 系统调用开销极大, 已弃用。
#   如需跨机通信, 请升级至 CycloneDDS 0.10+ 或使用 FastDDS UDP transport。
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# ─── 自动检测 ROS2 发行版 ───
_ft_detect_ros2() {
    for d in humble foxy; do
        if [ -f "/opt/ros/$d/setup.bash" ]; then
            echo "$d"
            return
        fi
    done
    case "$(lsb_release -rs 2>/dev/null)" in
        20.04) echo "foxy" ;;
        22.04) echo "humble" ;;
        *)     echo "" ;;
    esac
}

ROS2_DISTRO=$(_ft_detect_ros2)
ROS2_SETUP="/opt/ros/$ROS2_DISTRO/setup.bash"

# ─── 1. 载入 ROS2 环境 ───
if [ -f "$ROS2_SETUP" ] && [ -z "$ROS_DISTRO" ]; then
    source "$ROS2_SETUP"
fi

# ─── 2. 强制使用 FastDDS — 禁止 CycloneDDS ───
# FastDDS 是 ROS2 Foxy 默认 RMW, 内置零拷贝 SHM 传输,
# 32MB ADC 消息不走网络栈, 无 UDP 分片开销.
#
# CycloneDDS 0.7.0 无 SHM 支持, 同机大消息性能极差, 已禁止.
if [ -n "$RMW_IMPLEMENTATION" ] && [ "$RMW_IMPLEMENTATION" != "rmw_fastrtps_cpp" ]; then
    echo "[ft] ⚠ 检测到 RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}, 本框架仅支持 FastDDS"
    echo "[ft]    已自动切换为: RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    echo "[ft]    原因: CycloneDDS 0.7.0 无 SHM 传输, 32MB ADC 帧走 UDP→512分片→性能塌陷"
    echo ""
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 加载 FastDDS SHM 配置 (512MB 共享内存段, 适配 32MB ADC 帧, 无 UDP 回退)
FT_FASTDDS_XML="$PROJECT_ROOT/config/fastdds.xml"
if [ -f "$FT_FASTDDS_XML" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FT_FASTDDS_XML"
fi

# 显式清除 CycloneDDS 环境变量 (防止残留配置干扰)
unset CYCLONEDDS_URI

# ─── 清理上次运行的 SHM 僵尸段 (防止 /dev/shm 占满导致新节点创建失败) ───
# FastDDS 进程异常退出时不会清理 SHM 段文件, 多次运行累积后可能占满 /dev/shm
# (Jetson 默认 31GB), 导致新节点无法创建 SHM 段 → RTPS Participant 创建失败 → 节点崩溃.
# 在每次 source env.sh 时清理, 当前运行的进程的 SHM 段不受影响 (进程持有 fd).
FT_SHM_CLEANED=0
for f in /dev/shm/fastrtps_*; do
    [ -e "$f" ] || continue
    rm -f "$f" 2>/dev/null && FT_SHM_CLEANED=$((FT_SHM_CLEANED + 1))
done
if [ "$FT_SHM_CLEANED" -gt 0 ]; then
    echo "[ft] 已清理 ${FT_SHM_CLEANED} 个僵尸 SHM 段 (上次运行残留)"
fi

# ─── 3. 载入工作空间 ───
FT_SETUP="$PROJECT_ROOT/install/setup.bash"
if [ -f "$FT_SETUP" ]; then
    source "$FT_SETUP"
    echo "[ft] ROS2 $ROS2_DISTRO + FastDDS (SHM 512MB) + FT 工作空间已就绪"
else
    echo "[ft] ROS2 $ROS2_DISTRO + FastDDS (SHM 512MB) 已就绪 (工作空间未构建)"
fi

# ─── 4. 首次使用提示 ───
FT_BASHRC_MARKER="FT_Radar_env_loaded"
if [ -z "${!FT_BASHRC_MARKER}" ] && [ -f "$FT_SETUP" ]; then
    export FT_Radar_env_loaded=1
    if ! grep -q "scripts/env.sh" ~/.bashrc 2>/dev/null; then
        echo ""
        echo "  [TIP] 免 source 提示: 执行一次以下命令，之后每次开终端自动加载环境:"
        echo "     echo \"source $PROJECT_ROOT/scripts/env.sh\" >> ~/.bashrc"
        echo ""
    fi
fi
