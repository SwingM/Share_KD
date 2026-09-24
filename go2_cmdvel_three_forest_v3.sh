#!/usr/bin/env bash
# go2_cmdvel_three_forest_v3.sh — 三狗 RL + forest_v3 世界
# 用法: bash /home/ubuntu/go2_target_seek_delivery_0901_singledog_rl/go2_cmdvel_three_forest_v3.sh

W=/home/ubuntu/go2_target_seek_delivery/go2_policy_deployment
PTY=$W/scripts/run_rl_sim_pty.py
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG=$SCRIPT_DIR/.log
mkdir -p "$LOG"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD=/home/ubuntu/go2_target_seek_delivery_0901/KD_MODEL/world/forestV3.world
SPAWN_ROLL=0
SPAWN_Z=0.5
DOGS=(1 2 3)

export GAZEBO_MODEL_PATH="/home/ubuntu/go2_target_seek_delivery_0901/KD_MODEL/models:/home/ubuntu/go2_target_seek_delivery_0901/QY_MODEL/models:$HOME/.gazebo/models"
source /home/ubuntu/go2_target_seek_delivery/setup_gpu.sh >/dev/null 2>&1
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profile.xml"

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

echo "[3/9] 生成三只狗 (go2_1/go2_2/go2_3)..."
setsid bash -c "export AMENT_PREFIX_PATH='' && source /opt/ros/humble/setup.bash && \
    source $W/install/setup.bash && \
    SPAWN_DOGS=3 ros2 launch rl_sar spawn_dogs.launch.py spawn_roll:=$SPAWN_ROLL spawn_z:=$SPAWN_Z use_camera:=false >> $LOG/gazebo.log 2>&1" >/dev/null 2>&1 &
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

echo "[5/9] 逐狗初始化 (R→0→1→N)..."

stand_up_dog() {
    local i=$1
    echo "  [go2_$i] 发送 R 重置..."
    printf 'R' > $LOG/keys_$i
    sleep 4

    echo "  [go2_$i] 发送 0 起身..."
    printf '0' > $LOG/keys_$i
    sleep 6

    # 等待 Getting up 完成
    for _ in $(seq 1 15); do
        if tail -c 2000 $LOG/rlsim_$i.log | grep -qa "RLFSMStateRLLocomotion"; then
            echo "  [go2_$i] 已起身并进入策略"
            break
        fi
        sleep 1
    done

    echo "  [go2_$i] 发送 1 进入策略..."
    printf '1' > $LOG/keys_$i
    sleep 5
    if ! tail -c 2000 $LOG/rlsim_$i.log | grep -qa "RLFSMStateRLLocomotion"; then
        printf '1' > $LOG/keys_$i; sleep 4
    fi

    echo "  [go2_$i] 发送 N 开启 navigation..."
    # 等 rl_sim 完全就绪再发 N
    for _ in $(seq 1 10); do
        if tail -c 2000 $LOG/rlsim_$i.log | grep -qa "RLFSMStateRLLocomotion"; then
            break
        fi
        sleep 1
    done
    printf 'N' > $LOG/keys_$i
    sleep 3
    if ! tail -c 500 $LOG/rlsim_$i.log | grep -qa "Navigation mode: ON"; then
        printf 'N' > $LOG/keys_$i; sleep 3
    fi

    printf ' ' > $LOG/keys_$i

    # 等待 controller 激活（最多30秒）
    echo "  [go2_$i] 等待 controller 激活..."
    for _ in $(seq 1 15); do
        CTRL=$(timeout 5 ros2 control list_controllers -c /go2_$i/controller_manager 2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | grep -c "robot_joint_controller_go2_$i.*active")
        if [ "$CTRL" -ge 1 ]; then
            echo "  [go2_$i] ✅ controller active"
            return 0
        fi
        sleep 2
    done
    echo "  [go2_$i] ❌ controller 未能激活"
    return 1
}

for i in "${DOGS[@]}"; do
    stand_up_dog $i
    sleep 2
done

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

echo "[9/9] 最终验证..."
ALL_OK=true
for i in "${DOGS[@]}"; do
    CTRL=$(timeout 5 ros2 control list_controllers -c /go2_$i/controller_manager 2>/dev/null | grep -c "robot_joint_controller_go2_$i.*active")
    NAV=$(tail -c 500 $LOG/rlsim_$i.log | grep -c "Navigation mode: ON")
    ODOM=$(timeout 3 ros2 topic echo /go2_$i/odom --once --qos-reliability best_effort 2>/dev/null | grep -c "position:")
    if [ "$CTRL" -ge 1 ] && [ "$NAV" -ge 1 ] && [ "$ODOM" -ge 1 ]; then
        echo "  go2_$i: ✅ 全部就绪"
    else
        echo "  go2_$i: ❌ controller=$CTRL nav=$NAV odom=$ODOM"
        ALL_OK=false
    fi
done

if [ "$ALL_OK" = false ]; then
    echo ""
    echo "!! 部分狗未就绪，尝试重新初始化失败的狗..."
    for i in "${DOGS[@]}"; do
        CTRL=$(timeout 5 ros2 control list_controllers -c /go2_$i/controller_manager 2>/dev/null | grep -c "robot_joint_controller_go2_$i.*active")
        NAV=$(tail -c 500 $LOG/rlsim_$i.log | grep -c "Navigation mode: ON")
        ODOM=$(timeout 3 ros2 topic echo /go2_$i/odom --once --qos-reliability best_effort 2>/dev/null | grep -c "position:")
        if [ "$CTRL" -lt 1 ] || [ "$NAV" -lt 1 ] || [ "$ODOM" -lt 1 ]; then
            echo "  重试 go2_$i..."
            stand_up_dog $i
            sleep 3
        fi
    done
fi
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
echo "      三狗建图导航栈已启动"

echo ""
echo "============================================================"
echo "三狗 forest_v3 已就绪"
echo "  go2_1 话题: /go2_1/xxx"
echo "  go2_2 话题: /go2_2/xxx"
echo "  go2_3 话题: /go2_3/xxx"
echo "============================================================"
