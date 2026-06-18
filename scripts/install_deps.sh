#!/bin/bash
# ============================================================================
# FT Radar Framework — 系统依赖一键安装脚本
# ============================================================================
# 自动根据 Ubuntu 版本选择 ROS2 发行版:
#   Ubuntu 20.04 → ROS2 Foxy   (官方原生支持)
#   Ubuntu 22.04 → ROS2 Humble (官方原生支持)
#
# 两个发行版的 rclpy API 兼容，项目代码无需任何修改。
#
# 用法:
#   bash scripts/install_deps.sh                     # 安装项目依赖（ROS2 已安装时）
#   bash scripts/install_deps.sh --with-ros2         # 含 ROS2 本体安装（首次）
#   bash scripts/install_deps.sh --with-cyclonedds    # 安装 CycloneDDS (大消息传输优化)
#   bash scripts/install_deps.sh --dry-run           # 仅显示将安装的包，不执行
#
# 作者: zhengyuan.liu
# 日期: 2026.6.10
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

DRY_RUN=false
WITH_ROS2=false
WITH_CYCLONE=false

for arg in "$@"; do
    case "$arg" in
        --dry-run)          DRY_RUN=true ;;
        --with-ros2)        WITH_ROS2=true ;;
        --with-cyclonedds)  WITH_CYCLONE=true ;;
    esac
done

# ============================================================================
# 0. 检测系统信息 → 确定 ROS2 发行版
# ============================================================================

UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
UBUNTU_CODENAME=$(lsb_release -cs 2>/dev/null || echo "unknown")
UBUNTU_ARCH=$(dpkg --print-architecture 2>/dev/null || echo "amd64")

case "$UBUNTU_VERSION" in
    20.04)
        ROS2_DISTRO="foxy"
        ;;
    22.04)
        ROS2_DISTRO="humble"
        ;;
    *)
        echo "[WARN] 未识别的 Ubuntu 版本: $UBUNTU_VERSION"
        echo "  支持的版本: 20.04 (Foxy) / 22.04 (Humble)"
        echo "  请设置环境变量手动指定: export FT_ROS2_DISTRO=humble"
        if [ -n "$FT_ROS2_DISTRO" ]; then
            ROS2_DISTRO="$FT_ROS2_DISTRO"
            echo "  → 使用 FT_ROS2_DISTRO=$ROS2_DISTRO"
        else
            exit 1
        fi
        ;;
esac

ROS2_SETUP="/opt/ros/$ROS2_DISTRO/setup.bash"

echo "=============================================="
echo "  FT Radar Framework — 依赖安装"
echo "  Ubuntu:   $UBUNTU_VERSION ($UBUNTU_CODENAME)"
echo "  ROS2:     $ROS2_DISTRO"
echo "=============================================="

# ── 辅助函数 ──

_dpkg_installed() {
    dpkg -l "$1" 2>/dev/null | grep -q '^ii'
}

_apt_install() {
    local pkg="$1"
    if _dpkg_installed "$pkg"; then
        echo "  [OK] $pkg (已安装)"
        return 0
    fi
    echo "  [INSTALL] $pkg → 安装中..."
    if $DRY_RUN; then
        return 0
    fi
    if sudo apt-get install -y "$pkg"; then
        return 0
    else
        echo "     [FAIL] 失败！请检查网络和 apt 源"
        return 1
    fi
}

# ============================================================================
# 步骤 1: 配置 apt 源
# ============================================================================

echo ""
echo "[1/4] 配置 apt 源..."

# Ubuntu 20.04 默认不含 ROS2 apt 源 → 添加 packages.ros.org
if [ "$UBUNTU_VERSION" = "20.04" ]; then
    ROS2_REPO_FILE="/etc/apt/sources.list.d/ros2.list"
    if [ -f "$ROS2_REPO_FILE" ]; then
        echo "  [OK] ROS2 apt 源 (已配置)"
    else
        echo "  → Ubuntu 20.04: 添加 packages.ros.org apt 源..."
        if $DRY_RUN; then
            echo "     (dry-run) 将添加 ROS2 apt 源"
        else
            sudo apt-get update -qq || true
            sudo apt-get install -y curl gnupg2 software-properties-common || {
                echo "  ❌ 安装必要工具失败"; exit 1;
            }
            if ! sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
                -o /usr/share/keyrings/ros-archive-keyring.gpg; then
                echo "  ❌ 下载 ROS2 apt key 失败，请检查网络"
                exit 1
            fi
            echo "deb [arch=$UBUNTU_ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
                | sudo tee "$ROS2_REPO_FILE" > /dev/null
            echo "  [OK] ROS2 apt 源已添加"
        fi
    fi
else
    echo "  [OK] Ubuntu 22.04: ROS2 在默认仓库中"
fi

# 刷新包列表
if ! $DRY_RUN; then
    echo -n "  → 更新包索引..."
    if sudo apt-get update -qq 2>/dev/null; then
        echo " ✓"
    else
        echo " [WARN] (可忽略)"
    fi
fi

# ============================================================================
# 步骤 2: 系统基础包 + ROS2 功能包
# ============================================================================

echo ""
echo "[2/4] 系统包..."

SYSTEM_PACKAGES=(
    python3-pip
    python3-numpy
    python3-yaml
    python3-opencv
    python3-colcon-common-extensions
)

for pkg in "${SYSTEM_PACKAGES[@]}"; do
    _apt_install "$pkg"
done

echo ""
echo "[3/4] ROS2 $ROS2_DISTRO..."

# ── ROS2 本体 ──
ROS2_DESKTOP_PKG="ros-$ROS2_DISTRO-desktop"

if [ -f "$ROS2_SETUP" ]; then
    echo "  [OK] ROS2 $ROS2_DISTRO 已安装"
else
    if $WITH_ROS2; then
        echo "  [INSTALL] 安装 ROS2 $ROS2_DISTRO ($ROS2_DESKTOP_PKG)..."
        echo "     (约需 5-15 分钟，请耐心等待)"
        if $DRY_RUN; then
            echo "     (dry-run) 将安装 $ROS2_DESKTOP_PKG"
        else
            if ! sudo apt-get install -y "$ROS2_DESKTOP_PKG"; then
                echo ""
                echo "  ╔══════════════════════════════════════════╗"
                echo "  ║  ❌ ROS2 安装失败                         ║"
                echo "  ╠══════════════════════════════════════════╣"
                echo "  ║  诊断:                                    ║"
                echo "  ║    df -h /opt                             ║"
                echo "  ║    apt-cache search $ROS2_DESKTOP_PKG     ║"
                if [ "$UBUNTU_VERSION" = "20.04" ]; then
                echo "  ║    cat /etc/apt/sources.list.d/ros2.list  ║"
                fi
                echo "  ╚══════════════════════════════════════════╝"
                exit 1
            fi
            # 验证
            if [ -f "$ROS2_SETUP" ]; then
                echo "  [OK] ROS2 $ROS2_DISTRO 安装成功"
            else
                echo "  [WARN] 未找到 $ROS2_SETUP，但 apt 报告成功"
                echo "    请检查: ls -la /opt/ros/$ROS2_DISTRO/"
            fi
        fi
    else
        echo "  ❌ ROS2 $ROS2_DISTRO 未安装"
        echo ""
        echo "  请运行:  bash scripts/install_deps.sh --with-ros2"
        echo "  或手动:  https://docs.ros.org/en/$ROS2_DISTRO/Installation.html"
        exit 1
    fi
fi

# ── ROS2 功能包 ──
ROS2_PACKAGES=(
    "ros-$ROS2_DISTRO-cv-bridge"
    "ros-$ROS2_DISTRO-tf2-ros"
    "ros-$ROS2_DISTRO-rviz2"
)

# CycloneDDS — 大消息 (32MB+) 可靠传输; FastDDS UDP 分片 >500 片/帧会大量丢包
if $WITH_CYCLONE; then
    ROS2_PACKAGES+=("ros-$ROS2_DISTRO-rmw-cyclonedds-cpp")
fi

for pkg in "${ROS2_PACKAGES[@]}"; do
    _apt_install "$pkg"
done

# ============================================================================
# 步骤 3: 验证
# ============================================================================

echo ""
echo "[4/4] 验证关键模块..."

_verify_py() {
    local mod="$1" label="$2"
    if python3 -c "import $mod" 2>/dev/null; then
        echo "  [OK] $label"
    else
        echo "  [WARN] $label — source $ROS2_SETUP 后即可用"
    fi
}

_verify_py "numpy"  "numpy"
_verify_py "cv2"    "OpenCV (cv2)"
_verify_py "yaml"   "PyYAML"

if [ -f "$ROS2_SETUP" ]; then
    if bash -c "source $ROS2_SETUP 2>/dev/null && python3 -c 'import rclpy; import cv_bridge' 2>/dev/null"; then
        echo "  [OK] rclpy + cv_bridge (ROS2 $ROS2_DISTRO)"
    else
        echo "  [WARN] rclpy / cv_bridge 验证失败 — 请手动检查 ROS2 安装"
    fi
fi

# ============================================================================
# 完成
# ============================================================================

echo ""
echo "=============================================="
if $DRY_RUN; then
    echo "  [OK] 检查完成 (--dry-run)"
else
    echo "  [OK] 依赖安装完成！"
fi
echo "=============================================="
echo ""
echo "  项目构建:"
echo "    bash scripts/build.sh"
echo ""
echo "  每次新终端加载环境:"
echo "    source /opt/ros/$ROS2_DISTRO/setup.bash"
echo "    source install/setup.bash"
echo ""
echo "  启动:"
echo "    ros2 launch ft_framework ft_radar_launch.py"
echo "=============================================="
