#!/usr/bin/env bash
# explore_box_two.sh — 双狗分别探索不同区域
# 用法: bash explore_box_two.sh [box_id_1] [box_id_2]
# 示例: bash explore_box_two.sh 3 6  (go2_1探索box3, go2_2探索box6)

BOX_ID_1=${1:-3}
BOX_ID_2=${2:-6}
BOX_JSON="/home/ubuntu/go2_target_seek_delivery_0901/shadow_building_boxes.json"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/setup.bash 2>/dev/null
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash 2>/dev/null
export ROS_LOCALHOST_ONLY=1

echo "=========================================="
echo "  双狗探索: go2_1 -> Box $BOX_ID_1, go2_2 -> Box $BOX_ID_2"
echo "=========================================="

echo "等待 Nav2 完全就绪 (15s)..."
sleep 15

# 启动 go2_1 探索
echo "[1/2] 启动 go2_1 探索 Box $BOX_ID_1..."
python3 "$SCRIPT_DIR/explore_box.py" \
    --box-json "$BOX_JSON" \
    --box-id "$BOX_ID_1" \
    --robot-name go2_1 \
    --action-server /go2_1/navigate_to_pose \
    --map-topic /go2_1/map \
    --save-prefix "$SCRIPT_DIR/.log/explored_box_${BOX_ID_1}_go2_1" &
PID1=$!

# 启动 go2_2 探索
echo "[2/2] 启动 go2_2 探索 Box $BOX_ID_2..."
python3 "$SCRIPT_DIR/explore_box.py" \
    --box-json "$BOX_JSON" \
    --box-id "$BOX_ID_2" \
    --robot-name go2_2 \
    --action-server /go2_2/navigate_to_pose \
    --map-topic /go2_2/map \
    --save-prefix "$SCRIPT_DIR/.log/explored_box_${BOX_ID_2}_go2_2" &
PID2=$!

echo ""
echo "双狗探索已启动 (Ctrl+C 停止)"
echo "  go2_1 PID: $PID1"
echo "  go2_2 PID: $PID2"

trap "kill $PID1 $PID2 2>/dev/null; exit" INT TERM
wait $PID1 $PID2
