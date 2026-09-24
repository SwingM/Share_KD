#!/usr/bin/env bash
# explore_simple.sh — 简化版单狗探索（带初始化检查）
# 用法: bash explore_simple.sh [box_id]

BOX_ID=${1:-6}
BOX_JSON="/home/ubuntu/go2_target_seek_delivery_0901/shadow_building_boxes.json"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/setup.bash 2>/dev/null
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash 2>/dev/null
export ROS_LOCALHOST_ONLY=1

echo "=========================================="
echo "  简化版单狗探索 Box $BOX_ID"
echo "=========================================="

echo ""
echo "[0/1] 检查狗的状态..."
CTRL=$(timeout 8 ros2 control list_controllers -c /go2_1/controller_manager 2>/dev/null | grep -c "robot_joint_controller_go2_1.*active")
NAV=$(tail -c 500 "$SCRIPT_DIR/.log/rlsim_1.log" 2>/dev/null | grep -c "Navigation mode: ON")
ODOM=$(timeout 3 ros2 topic echo /go2_1/odom --once --qos-reliability best_effort 2>/dev/null | grep -c "position:")
if [ "$CTRL" -ge 1 ] && [ "$NAV" -ge 1 ] && [ "$ODOM" -ge 1 ]; then
    echo "  go2_1: ✅"
else
    echo "  go2_1: ❌ CTRL=$CTRL NAV=$NAV ODOM=$ODOM"
    echo "!! 请先运行初始化脚本"
    read -p "强制继续? (y/N) " -n 1 -r; echo
    [[ ! $REPLY =~ ^[Yy]$ ]] && exit 1
fi

echo "等待 Nav2 就绪 (10s)..."
sleep 10

python3 "$SCRIPT_DIR/explore_simple.py" \
    --box-json "$BOX_JSON" --box-id "$BOX_ID" --robot-name go2_1 \
    --action-server /go2_1/navigate_to_pose --map-topic /go2_1/map \
    --save-prefix "$SCRIPT_DIR/.log/explored_box_${BOX_ID}"
