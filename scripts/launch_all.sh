#!/bin/bash
# ============================================================================
# FT Radar Framework — 一键启动脚本 (兼容入口)
# ============================================================================
# 直接委托到 scripts/start.sh (推荐入口).
#
# 用法:
#   bash scripts/launch_all.sh                   # 默认 cuda + C++ rx
#   bash scripts/launch_all.sh python            # Python RSP + C++ rx
#   bash scripts/launch_all.sh cuda --rviz       # CUDA + RViz
#   bash scripts/launch_all.sh both_compare      # 双路对比
#
# 作者: zhengyuan.liu
# 日期: 2026.6.10  (更新 2026.6.18)
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/start.sh" "$@"
