#!/bin/bash
# Start single Go2 with RL policy controller + RTAB-Map + Nav2
# Usage: ./start_go2_1_rl_mapping_nav.sh [scene]
# scene: city (default), forest, airport

set -eo pipefail

SCENE="${1:-city}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$PROJECT_DIR/go2_ws_v2"

# ---- Environment setup ----
source /opt/ros/humble/setup.bash
cd "$WS_DIR"
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "[WARN] install/setup.bash not found, building workspace..."
    colcon build --symlink-install
    source install/setup.bash
fi

export DELIVERY_ROOT="$PROJECT_DIR"
export QY_MODEL_ROOT="$DELIVERY_ROOT/QY_MODEL"
export KD_MODEL_ROOT="$DELIVERY_ROOT/KD_MODEL"
export GAZEBO_MODEL_PATH="$QY_MODEL_ROOT/models:$KD_MODEL_ROOT/models:${GAZEBO_MODEL_PATH:-}"

echo "=== Phase 1: Launch Gazebo world ==="
ros2 launch go2_config gazebo_target_seek_world.launch.py &
GAZEBO_PID=$!

echo "Waiting for /clock topic..."
timeout 60 bash -c 'until ros2 topic echo /clock --once 2>/dev/null; do sleep 1; done'
echo "Gazebo clock ready, waiting 10s for world to settle..."
sleep 10

echo "=== Phase 2: Spawn Go2 with RL controller ==="
ros2 launch rl_go2_controller spawn_go2_velodyne_rl.launch.py \
    scene:="$SCENE" \
    use_sim_time:=true \
    use_ground_truth_odom:=true \
    enable_lidar:=true \
    enable_camera:=false &
SPAWN_PID=$!

echo "Waiting for /go2_1/velodyne_points and /go2_1/odom..."
timeout 90 bash -c '
until ros2 topic echo /go2_1/velodyne_points --once 2>/dev/null; do sleep 2; done
until ros2 topic echo /go2_1/odom --once 2>/dev/null; do sleep 2; done
'
echo "Robot ready, waiting 5s for controllers to stabilize..."
sleep 5

echo "=== Phase 3: Launch RTAB-Map + Nav2 ==="
ros2 launch go2_mapping_nav go2_1_mapping_nav.launch.py \
    use_sim_time:=true \
    use_merged_map:=false \
    use_rviz:=true \
    delete_db_on_start:=true

# Cleanup
kill $SPAWN_PID $GAZEBO_PID 2>/dev/null || true
