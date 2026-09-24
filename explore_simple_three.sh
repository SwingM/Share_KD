#!/usr/bin/env bash
BOX_ID_1=${1:-3}
BOX_ID_2=${2:-6}
BOX_ID_3=${3:-5}
BOX_JSON="/home/ubuntu/go2_target_seek_delivery_0901/shadow_building_boxes.json"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /home/ubuntu/go2_target_seek_delivery/go2_policy_deployment/install/setup.bash 2>/dev/null
source /home/ubuntu/go2_target_seek_delivery/go2_ws_v2/install/setup.bash 2>/dev/null
export ROS_LOCALHOST_ONLY=1
SCRIPT_DIR2="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR2/fastdds_profile.xml"

echo "=========================================="
echo "  Simple 3-dog explore: go2_1->Box$BOX_ID_1  go2_2->Box$BOX_ID_2  go2_3->Box$BOX_ID_3"
echo "=========================================="

echo ""
echo "[0/3] Checking dogs..."
ALL_OK=true
for i in 1 2 3; do
    echo "--- go2_$i ---"
    CTRL_LIST=$(timeout 5 ros2 control list_controllers -c /go2_$i/controller_manager 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g')
    CTRL_MY=$(echo "$CTRL_LIST" | grep -c "robot_joint_controller_go2_$i.*active")
    NAV=$(tail -c 500 /tmp/opencode/rlsim_$i.log 2>/dev/null | grep -c "Navigation mode: ON")
    ODOM=$(timeout 3 ros2 topic echo /go2_$i/odom --once --qos-reliability best_effort 2>/dev/null | grep -c "position:")

    if [ "$CTRL_MY" -ge 1 ]; then echo "  OK controller active"; else echo "  FAIL controller"; ALL_OK=false; fi
    if [ "$NAV" -ge 1 ]; then echo "  OK nav ON"; else echo "  FAIL nav"; ALL_OK=false; fi
    if [ "$ODOM" -ge 1 ]; then echo "  OK odom"; else echo "  FAIL odom"; ALL_OK=false; fi
done

if [ "$ALL_OK" = false ]; then
    echo ""
    echo "!! Dogs not ready. Run go2_cmdvel_three_forest_v3.sh first."
    read -p "Force continue? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then exit 1; fi
fi

echo ""
echo "[1/3] Wait for Nav2 (15s)..."
sleep 15

echo "[2/3] Check map..."
for i in 1 2 3; do
    if timeout 3 ros2 topic info /go2_$i/map >/dev/null 2>&1; then echo "  go2_$i/map OK"; else echo "  go2_$i/map MISSING"; fi
done

echo "[3/3] Starting exploration..."
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

python3 "$SCRIPT_DIR/explore_simple.py" \
    --box-json "$BOX_JSON" --box-id "$BOX_ID_3" --robot-name go2_3 \
    --action-server /go2_3/navigate_to_pose --map-topic /go2_3/map \
    --save-prefix "/tmp/opencode/explored_box_${BOX_ID_3}_go2_3" &
PID3=$!

echo "Exploration started (Ctrl+C to stop)"
trap "kill $PID1 $PID2 $PID3 2>/dev/null; exit" INT TERM
wait $PID1 $PID2 $PID3
