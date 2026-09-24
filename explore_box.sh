#!/usr/bin/env bash
# explore_box.sh - 探索指定box区域的未知区域
# 
# 用法:
#   bash explore_box.sh [box_id]
#
# 示例:
#   bash explore_box.sh 8    # 探索box 8 (很小: 2.2m x 0.88m)
#   bash explore_box.sh 6    # 探索box 6 (推荐: 25.4m x 20.9m)
#   bash explore_box.sh 3    # 探索box 3 (大: 32.77m x 28.16m)

BOX_ID=${1:-8}
BOX_JSON="/home/ubuntu/go2_target_seek_delivery_0901/shadow_building_boxes.json"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/setup.bash 2>/dev/null
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash 2>/dev/null
export ROS_LOCALHOST_ONLY=1

echo "=========================================="
echo "  探索 Box $BOX_ID 未知区域"
echo "=========================================="
echo ""

echo "等待 Nav2 完全就绪 (10s)..."
sleep 10

echo "Box JSON: $BOX_JSON"
echo "Robot: go2_1"
echo "Action Server: /navigate_to_pose"
echo ""

python3 "$SCRIPT_DIR/explore_box.py" \
    --box-json "$BOX_JSON" \
    --box-id "$BOX_ID" \
    --robot-name go2_1 \
    --action-server /go2_1/navigate_to_pose \
    --map-topic /go2_1/map \
    --save-prefix "$SCRIPT_DIR/.log/explored_box_${BOX_ID}"
