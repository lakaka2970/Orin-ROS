#!/bin/bash
# ============================================================================
# FT Framework — 一键构建脚本
# ============================================================================
# 用法：
#   bash scripts/build.sh           # 增量构建
#   bash scripts/build.sh --clean   # 清理后重新构建
#
# 作者：zhengyuan.liu
# 日期：2026.6.8
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "  FT Framework — 构建脚本"
echo "  项目根目录: $PROJECT_ROOT"
echo "=============================================="

# 加载 ROS2 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] ROS2 Humble 环境已加载"
else
    echo "[ERROR] 未找到 /opt/ros/humble/setup.bash"
    echo "        请确认 ROS2 Humble 已安装"
    exit 1
fi

cd "$PROJECT_ROOT"

# 清理模式
if [ "$1" = "--clean" ]; then
    echo "[INFO] 清理旧的构建产物..."
    rm -rf build/ install/ log/
    echo "[OK] 清理完成"
fi

# 构建
echo "[INFO] 开始构建 ft_framework..."
colcon build --packages-select ft_framework --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "=============================================="
    echo "  构建成功！"
    echo ""
    echo "  下一步："
    echo "    source install/setup.bash"
    echo "    ros2 launch ft_framework ft_framework.launch.py"
    echo "=============================================="
else
    echo ""
    echo "[ERROR] 构建失败，请检查错误信息"
    exit 1
fi
