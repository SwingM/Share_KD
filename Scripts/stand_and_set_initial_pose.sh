#!/bin/bash
# ----------------------------------------------------------------------------
# 一键：让狗站起来 + 向 Nav2 确认初始位姿
#
# 用法:  ./stand_and_set_initial_pose.sh
#
# 功能:
#   1) 发布 /go2_1/stand_cmd = true  -> RL 控制器从"趴着待机"进入"站立行走"模式
#   2) 读取狗当前 map 位姿 (TF)     -> 发布 /go2_1/initialpose 给 Nav2
#
# 前提: 系统已由 start_go2_1_rl_mapping_nav.sh 启动
# ----------------------------------------------------------------------------
set -e

WS=/home/ubuntu/go2_target_seek_delivery/go2_ws_v2
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash

NS="go2_1"

echo ">>> 1) 让狗站起来 (stand_cmd)"
ros2 topic pub -1 /$NS/stand_cmd std_msgs/msg/Bool "{data: true}" 2>&1 | sed 's/^/    /'
echo "    done"

echo ">>> 2) 读取狗当前 map 位姿 (TF)"
# 用 tf2_echo 解析出 map -> base_footprint 的平移/旋转
TF_OUT=$(timeout 5 ros2 run tf2_ros tf2_echo $NS/map $NS/base_footprint 2>/dev/null | grep -E 'Translation|Rotation' | head -2)
echo "    $TF_OUT"

# 提取 x, y
X=$(echo "$TF_OUT" | grep Translation | sed -E 's/.*\[(-?[0-9.]+), (-?[0-9.]+),.*/\1/')
Y=$(echo "$TF_OUT" | grep Translation | sed -E 's/.*\[(-?[0-9.]+), (-?[0-9.]+),.*/\2/')

if [ -z "$X" ] || [ -z "$Y" ]; then
    echo "    [错误] 无法读取狗位姿，请确认系统已启动且 TF 存在"
    exit 1
fi
echo "    map 位姿: x=$X, y=$Y"

# 从 TF 的 Rotation 四元数直接取 z/w (已是 map 坐标系下的朝向)
QZ=$(echo "$TF_OUT" | grep Rotation | sed -E 's/.*\[(-?[0-9.]+), (-?[0-9.]+), (-?[0-9.]+), (-?[0-9.]+)\].*/\3/')
QW=$(echo "$TF_OUT" | grep Rotation | sed -E 's/.*\[(-?[0-9.]+), (-?[0-9.]+), (-?[0-9.]+), (-?[0-9.]+)\].*/\4/')
echo "    quaternion (z,w) = ($QZ, $QW)"

echo ">>> 3) 发布初始位姿到 Nav2 (/go2_1/initialpose)"
ros2 topic pub -1 /$NS/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: $NS/map},
  pose: {pose: {position: {x: $X, y: $Y, z: 0.0},
         orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}
}" 2>&1 | sed 's/^/    /'
echo "    done"

echo ""
echo "============================================================"
echo " 完成! 狗已站立，Nav2 初始位姿已设置。"
echo " 现在可在 Rviz 用 '2D Goal Pose' 发导航目标, 或用:"
echo "   ros2 action send_goal /go2_1/navigate_to_pose ..."
echo "============================================================"
