#!/bin/bash
# ============================================================================
# FT Radar Framework — 环境加载脚本
# ============================================================================
# 一个 source 完成:
#   1. 检测并载入 ROS2 发行版环境
#   2. 默认使用 FastDDS (内置 SHM 共享内存传输)
#   3. 载入工作空间 install/setup.bash
#
# 用法:
#   source scripts/env.sh
#
# 免 source 自动加载 (推荐):
#   echo "source ~/Orin-ROS/scripts/env.sh" >> ~/.bashrc
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

# ─── 2. 默认使用 FastDDS (内置 SHM 传输, 同机节点走共享内存) ───
# FastDDS 是 ROS2 Foxy 默认 RMW, 内置零拷贝 SHM 传输,
# 32MB ADC 消息不走网络栈, 无 UDP 分片开销.
#
# 可选覆盖:
#   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp bash scripts/start.sh ...
#   注意: CycloneDDS 0.7.0 无 SHM 支持, 仅推荐跨机通信时使用.
if [ -z "$RMW_IMPLEMENTATION" ]; then
    # 默认不设置 → ROS2 自动使用 FastDDS (rmw_fastrtps_cpp)
    :  # no-op
fi

# 加载 FastDDS SHM 配置 (128MB 共享内存段, 适配 32MB ADC 帧)
FT_FASTDDS_XML="$PROJECT_ROOT/config/fastdds.xml"
if [ -z "$RMW_IMPLEMENTATION" ] || [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ]; then
    if [ -f "$FT_FASTDDS_XML" ]; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="$FT_FASTDDS_XML"
    fi
fi

# ─── 3. 载入工作空间 ───
FT_SETUP="$PROJECT_ROOT/install/setup.bash"
if [ -f "$FT_SETUP" ]; then
    source "$FT_SETUP"
    echo "[ft] ROS2 $ROS2_DISTRO + ${RMW_IMPLEMENTATION:-FastDDS} + FT 工作空间已就绪"
else
    echo "[ft] ROS2 $ROS2_DISTRO + ${RMW_IMPLEMENTATION:-FastDDS} 已就绪 (工作空间未构建)"
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
