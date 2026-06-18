#!/bin/bash
# ============================================================================
# FT Radar Framework — 环境加载脚本
# ============================================================================
# 一个 source 完成:
#   1. 检测并载入 ROS2 发行版环境
#   2. 启用 CycloneDDS (大消息可靠传输)
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

# ─── 2. 启用 CycloneDDS (优先于 FastDDS) ───
# CycloneDDS 内置 SHM 传输, 32MB ADC 消息不再被 UDP 分片
# 若未安装 ros-foxy-rmw-cyclonedds-cpp, 回退到默认 FastDDS
if [ -f "/opt/ros/$ROS2_DISTRO/share/rmw_cyclonedds_cpp/local_setup.bash" ] || \
   [ -d "/opt/ros/$ROS2_DISTRO/share/rmw_cyclonedds_cpp" ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    # 显式加载 CycloneDDS XML 配置 (启用 SHM 传输, 避免 32MB 走 UDP 分片)
    FT_CYCLONE_XML="$PROJECT_ROOT/config/cyclonedds.xml"
    if [ -f "$FT_CYCLONE_XML" ]; then
        export CYCLONEDDS_URI="file://$FT_CYCLONE_XML"
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
