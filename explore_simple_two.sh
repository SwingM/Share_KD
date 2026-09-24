#!/usr/bin/env bash
# explore_simple_two.sh — 简化版双狗探索（带初始化检查）
# 用法: bash explore_simple_two.sh [box_id_1] [box_id_2]

BOX_ID_1=${1:-3}
BOX_ID_2=${2:-6}
BOX_JSON="/home/ubuntu/go2_target_seek_delivery_0901/shadow_building_boxes.json"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/setup.bash 2>/dev/null
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash 2>/dev/null
export ROS_LOCALHOST_ONLY=1

echo "=========================================="
echo "  简化版双狗探索: go2_1→Box$BOX_ID_1  go2_2→Box$BOX_ID_2"
echo "=========================================="

echo ""
echo "[0/2] 检查狗的状态..."
ALL_OK=true
for i in 1 2; do
    CTRL=$(timeout 8 ros2 control list_controllers -c /go2_$i/controller_manager 2>/dev/null | grep -c "robot_joint_controller_go2_$i.*active")
    NAV=$(tail -c 500 /tmp/opencode/rlsim_$i.log 2>/dev/null | grep -c "Navigation mode: ON")
    ODOM=$(timeout 3 ros2 topic echo /go2_$i/odom --once --qos-reliability best_effort 2>/dev/null | grep -c "position:")
    if [ "$CTRL" -ge 1 ] && [ "$NAV" -ge 1 ] && [ "$ODOM" -ge 1 ]; then
        echo "  go2_$i: ✅"
    else
        echo "  go2_$i: ❌ CTRL=$CTRL NAV=$NAV ODOM=$ODOM"
        ALL_OK=false
    fi
done
if [ "$ALL_OK" = false ]; then
    echo "!! 请先运行初始化脚本"
    read -p "强制继续? (y/N) " -n 1 -r; echo
    [[ ! $REPLY =~ ^[Yy]$ ]] && exit 1
fi

echo "等待 Nav2 就绪 (15s)..."
sleep 15

python3 "$SCRIPT_DIR/explore_simple.py" \
    --box-json "$BOX_JSON" --box-id "$BOX_ID_1" --robot-name go2_1 \
    --action-server /go2_1/navigate_to_pose --map-topic /go2_1/map \
    --save-prefix "/tmp/opencode/explored_box_${BOX_ID_1}_go2_1" &
PID1=$!

python3 "$SCRIPT_DIR/explore_simple.py" \
    --box-json "$BOX_JSON" --box-id "$BOX_ID_2" --robot-name go2_2 \
    --action-server /go2_2/navigate_to_pose --map-topic /go2_2/map \
    --save-prefix "/tmp/opencode/explored_box_${BOX_ID_2}_go2_2" &
PID2=$!

echo "探索已启动 (Ctrl+C 停止)"
trap "kill $PID1 $PID2 2>/dev/null; exit" INT TERM
wait $PID1 $PID2
