#!/usr/bin/env bash
# go2_cmdvel_two_forest_v3.sh — 双狗 RL + forest_v3 世界
# 用法: bash /home/ubuntu/go2_target_seek_delivery_0901_singledog_rl/go2_cmdvel_two_forest_v3.sh

W=/home/ubuntu/go2_target_seek_delivery/go2_policy_deployment
PTY=$W/scripts/run_rl_sim_pty.py
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG=$SCRIPT_DIR/.log
mkdir -p "$LOG"
WORLD=/home/ubuntu/go2_target_seek_delivery_0901/KD_MODEL/world/forestV3.world
SPAWN_ROLL=0
SPAWN_Z=0.5
DOGS=(1 2)

export GAZEBO_MODEL_PATH="/home/ubuntu/go2_target_seek_delivery_0901/KD_MODEL/models:/home/ubuntu/go2_target_seek_delivery_0901/QY_MODEL/models:$HOME/.gazebo/models"
source /home/ubuntu/go2_target_seek_delivery/setup_gpu.sh >/dev/null 2>&1
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_LOCALHOST_ONLY=1

wait_grep() {
    local t=0
    until grep -qa "$1" "$2" 2>/dev/null; do
        sleep 2; t=$((t+2))
        [ $t -ge $3 ] && { echo "WARN: 超时等待 [$1] ($2)"; return 1; }
    done
    return 0
}

echo "[1/9] 清理旧实例..."
pkill -9 -f "run_rl_sim_pty" 2>/dev/null
pkill -9 -f "rl_sar/rl_sim" 2>/dev/null
pkill -9 -f "ros2 launch rl_sar" 2>/dev/null
pkill -9 -f "explore_box" 2>/dev/null
sleep 1
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f parameter_blackboard 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null
pkill -9 -f "gt_odom_bridge" 2>/dev/null
pkill -9 -f "static_transform_publisher" 2>/dev/null
pkill -9 -f "rtabmap" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null
pkill -9 -f "bt_navigator" 2>/dev/null
pkill -9 -f "controller_server" 2>/dev/null
pkill -9 -f "planner_server" 2>/dev/null
pkill -9 -f "behavior_server" 2>/dev/null
pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f "ros2_daemon" 2>/dev/null
pkill -9 -f "_ros2_daemon" 2>/dev/null
rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null
rm -rf ~/.ros/daemon 2>/dev/null
sleep 3
if source /opt/ros/humble/setup.bash 2>/dev/null; then
    ros2 daemon stop >/dev/null 2>&1
    sleep 1
    ros2 daemon start >/dev/null 2>&1
    sleep 3
fi

echo "[2/9] 启动 forestV3 世界..."
: > $LOG/gazebo.log
setsid bash -c "source /opt/ros/humble/setup.bash && source $W/install/setup.bash && \
    ros2 launch rl_sar gazebo_world.launch.py world:=$WORLD > $LOG/gazebo.log 2>&1" >/dev/null 2>&1 &
until timeout 4 ros2 topic echo /clock --once --qos-reliability best_effort --qos-durability volatile --qos-history keep_last >/dev/null 2>&1; do sleep 3; done
echo "      世界已加载"

echo "[3/9] 生成两只狗 (go2_1/go2_2)..."
setsid bash -c "export AMENT_PREFIX_PATH='' && source /opt/ros/humble/setup.bash && \
    source $W/install/setup.bash && \
    SPAWN_DOGS=2 ros2 launch rl_sar spawn_dogs.launch.py spawn_roll:=$SPAWN_ROLL spawn_z:=$SPAWN_Z use_camera:=false >> $LOG/gazebo.log 2>&1" >/dev/null 2>&1 &
for i in "${DOGS[@]}"; do
    until timeout 3 ros2 topic list 2>/dev/null | grep -q "^/go2_$i/joint_states$"; do sleep 2; done
    echo "      go2_$i 已生成"
done
for i in "${DOGS[@]}"; do
    until timeout 3 ros2 service list 2>/dev/null | grep -q "/go2_$i/param_node/get_parameters"; do sleep 2; done
    echo "      go2_$i param_node 就绪"
done
for i in "${DOGS[@]}"; do
    ok=0
    for _ in $(seq 1 60); do
        if timeout 8 ros2 control list_controllers -c /go2_$i/controller_manager 2>/dev/null \
             | grep -E "robot_joint_controller_go2_$i" | grep -q active; then
            ok=1; break
        fi
        sleep 2
    done
    if [ "$ok" -eq 1 ]; then
        echo "      go2_$i robot_joint_controller 已 active"
    else
        echo "      !! go2_$i robot_joint_controller 未能 active" >&2
    fi
done

echo "[4/9] 启动 RL 策略进程..."
for i in "${DOGS[@]}"; do
    : > $LOG/rlsim_$i.log
    if [ ! -p "$LOG/keys_$i" ]; then rm -f "$LOG/keys_$i"; mkfifo "$LOG/keys_$i"; fi
    setsid python3 "$PTY" $i >/dev/null 2>&1 &
    wait_grep "RL_Sim start" $LOG/rlsim_$i.log 90
    echo "      rl_sim(go2_$i) 已就绪"
done

echo "[5/9] 发送 R 重置..."
for i in "${DOGS[@]}"; do printf 'R' > $LOG/keys_$i; done
sleep 5

echo "[6/9] 双狗自动起身..."
for i in "${DOGS[@]}"; do printf '0' > $LOG/keys_$i; done
sleep 10

echo "[7/9] 进入策略模式 + 开启 navigation_mode..."
for i in "${DOGS[@]}"; do printf '1' > $LOG/keys_$i; done
sleep 6
for i in "${DOGS[@]}"; do
    if tail -c 5000 $LOG/rlsim_$i.log | grep -qa "RLFSMStateRLLocomotion"; then
        echo "      go2_$i 已进入 RL 行走策略"
    else
        printf '1' > $LOG/keys_$i; sleep 4
        echo "      go2_$i 重试进入策略"
    fi
done
for i in "${DOGS[@]}"; do printf 'N' > $LOG/keys_$i; done
sleep 3
for i in "${DOGS[@]}"; do
    if grep -qa "Navigation mode: ON" $LOG/rlsim_$i.log; then
        echo "      go2_$i navigation_mode: ON"
    else
        printf 'N' > $LOG/keys_$i; sleep 2
        echo "      go2_$i navigation_mode: 已重试"
    fi
done
for i in "${DOGS[@]}"; do printf ' ' > $LOG/keys_$i; done

echo ""
echo "[8/9] 启动 odom 桥接 + static TF..."
WS2=/home/ubuntu/go2_target_seek_delivery/go2_ws_v2
for i in "${DOGS[@]}"; do
    nohup python3 "$W/scripts/gt_odom_bridge.py" --ros-args -p namespace:=go2_$i \
        > $LOG/gt_odom_bridge_$i.log 2>&1 &
    nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 go2_$i/base_link go2_$i/base >/dev/null 2>&1 &
    nohup ros2 run tf2_ros static_transform_publisher 0.2 0 0.1177 0 0 0 go2_$i/trunk go2_$i/velodyne >/dev/null 2>&1 &
done
sleep 3

echo "[9/9] 启动建图导航栈 (× 2)..."
for i in "${DOGS[@]}"; do
    setsid bash -c "source /opt/ros/humble/setup.bash && source $W/install/setup.bash && \
        source $WS2/install/setup.bash && export DELIVERY_ROOT=/home/ubuntu/go2_target_seek_delivery && \
        ros2 launch go2_mapping_nav go2_${i}_mapping_nav.launch.py \
            use_sim_time:=true use_merged_map:=false use_rviz:=$([ $i -eq 1 ] && echo true || echo false) \
            delete_db_on_start:=true > $LOG/mapping_nav_$i.log 2>&1" >/dev/null 2>&1 &
done
for i in "${DOGS[@]}"; do
    until timeout 3 ros2 topic info /go2_$i/scan >/dev/null 2>&1; do sleep 2; done
    until timeout 3 ros2 topic info /go2_$i/map >/dev/null 2>&1; do sleep 2; done
done
echo "      双狗建图导航栈已启动"

echo ""
echo "============================================================"
echo "双狗 forest_v3 已就绪"
echo "  go2_1 话题: /go2_1/xxx"
echo "  go2_2 话题: /go2_2/xxx"
echo "============================================================"
