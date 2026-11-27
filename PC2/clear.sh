#!/bin/bash
set -euo pipefail   # 可选：遇到错误即退出，防止继续执行

BASE="/home/action/code/ROBOCON2026_base"

# 删除所有 .pcd 文件
find "$BASE/pcd" "$BASE/pcd/source" "$BASE/pcd/target" -maxdepth 1 -type f -name '*.pcd' -delete 2>/dev/null || true

# 删除各记录目录下的 .txt 文件
for subdir in l515_data lidar mydebug logs/action_planner_node logs/controlnode logs/ICP logs/lidar_node logs/mcuNode; do
    find "$BASE/record/$subdir" -maxdepth 1 -type f -name '*.txt' -delete 2>/dev/null || true
done