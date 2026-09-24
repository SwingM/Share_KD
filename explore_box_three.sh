#!/usr/bin/env bash
# explore_box_three.sh — 三狗分别探索不同区域
# 用法: bash explore_box_three.sh [box_id_1] [box_id_2] [box_id_3]
# 示例: bash explore_box_three.sh 3 6 5  (go2_1→Box3, go2_2→Box6, go2_3→Box5)

BOX_ID_1=${1:-3}
BOX_ID_2=${2:-6}
BOX_ID_3=${3:-5}
BOX_JSON="/home/ubuntu/go2_target_seek_delivery_0901/shadow_building_boxes.json"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/setup.bash 2>/dev/null
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash 2>/dev/null
export ROS_LOCALHOST_ONLY=1

echo "=========================================="
echo "  三狗探索: go2_1→Box$BOX_ID_1  go2_2→Box$BOX_ID_2  go2_3→Box$BOX_ID_3"
echo "=========================================="

echo "等待 Nav2 完全就绪 (15s)..."
sleep 15

# go2_1
echo "[1/3] 启动 go2_1 探索 Box $BOX_ID_1..."
python3 "$SCRIPT_DIR/explore_box.py" \
    --box-json "$BOX_JSON" --box-id "$BOX_ID_1" --robot-name go2_1 \
    --action-server /go2_1/navigate_to_pose --map-topic /go2_1/map \
    --save-prefix "$SCRIPT_DIR/.log/explored_box_${BOX_ID_1}_go2_1" &
PID1=$!

# go2_2
echo "[2/3] 启动 go2_2 探索 Box $BOX_ID_2..."
python3 "$SCRIPT_DIR/explore_box.py" \
    --box-json "$BOX_JSON" --box-id "$BOX_ID_2" --robot-name go2_2 \
    --action-server /go2_2/navigate_to_pose --map-topic /go2_2/map \
    --save-prefix "$SCRIPT_DIR/.log/explored_box_${BOX_ID_2}_go2_2" &
PID2=$!

# go2_3
echo "[3/3] 启动 go2_3 探索 Box $BOX_ID_3..."
python3 "$SCRIPT_DIR/explore_box.py" \
    --box-json "$BOX_JSON" --box-id "$BOX_ID_3" --robot-name go2_3 \
    --action-server /go2_3/navigate_to_pose --map-topic /go2_3/map \
    --save-prefix "$SCRIPT_DIR/.log/explored_box_${BOX_ID_3}_go2_3" &
PID3=$!

echo ""
echo "三狗探索已启动 (Ctrl+C 停止)"
echo "  go2_1 PID: $PID1"
echo "  go2_2 PID: $PID2"
echo "  go2_3 PID: $PID3"

trap "kill $PID1 $PID2 $PID3 2>/dev/null; exit" INT TERM
wait $PID1 $PID2 $PID3
