#!/usr/bin/env bash
# view_explore.sh — 打开 RViz 看某只狗的探索过程
# 用法:
#   bash view_explore.sh        # 看 go2_1
#   bash view_explore.sh 3      # 看 go2_3
#
# 能看到：建好的地图 / 激光 / 机器人模型 / TF / Nav2 全局与局部路径
#         Explore Region (绿色半透明 = 它负责的分区范围)
#         Frontiers      (青色小球 = 当前候选前沿点)
#         Current Goal   (橙色大球 = 它此刻要去的点)

set +e
DOG=${1:-1}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash >/dev/null 2>&1
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash 

# 必须和仿真栈用同一套 DDS 配置，否则看不到任何话题
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profile.xml"
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

if [ -z "${DISPLAY:-}" ]; then
    echo "警告: 没有 DISPLAY，RViz 需要图形界面（你现在是开着 GUI=true 的，应该没问题）"
fi

exec rviz2 -d "$SCRIPT_DIR/rviz/explore_go2_$DOG.rviz"
