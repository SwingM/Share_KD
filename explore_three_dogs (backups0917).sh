#!/usr/bin/env bash
# explore_three_dogs.sh — 一键启动三狗探索（全自动）
# 用法: bash explore_three_dogs.sh
# 重要: 3 狗时 DDS participant 极多，FastDDS 的 SHM 传输会在 /dev/shm 建大量 port，
#       participant 多了出现 Failed init_port 并静默丢包(Nav2 lifecycle 永远停在 bringup)。
#       所以全部通信走 FASTRTPS UDPv4（见 fastdds_profile.xml），且必须 unset ROS_LOCALHOST_ONLY
#       （localhost-only 会把 participant 拉回 lo 并重新启用 SHM）。
#       检查工具一律用 Python rclpy，不用 ros2 CLI。

set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
W=/home/ubuntu/go2_target_seek_delivery/go2_policy_deployment
WS2=/home/ubuntu/go2_target_seek_delivery/go2_ws_v2
PTY=$W/scripts/run_rl_sim_pty.py
LOG=$SCRIPT_DIR/.log
export LOG
mkdir -p "$LOG"
# forestV3_light.world = 原版去掉 uav1 的 848x480 深度相机+点云（没人订阅，纯耗 gzserver）
# 想用回带相机的原版：WORLD=/home/ubuntu/.../forestV3.world bash explore_three_dogs.sh
WORLD=${WORLD:-/home/ubuntu/go2_target_seek_delivery_0901/KD_MODEL/world/forestV3_light.world}
DOGS=(1 2 3)
# 分区配置：默认 go2_1/2/3 分别探索 Box 3/6/5
#   BOX_IDS="1 2 4"  -> 改分区（数量要和狗数一致）
#   BOX_JSON=/path/my_boxes.json -> 用自己的分区文件
BOX_JSON=${BOX_JSON:-/home/ubuntu/go2_target_seek_delivery_0901/shadow_building_boxes.json}
BOX_IDS=${BOX_IDS:-"3 6 5"}
# 分区很大(几十米以上)时把这两个调一下：
#   MAX_GOAL_DISTANCE  前沿搜索半径(米)，默认 15；大分区用 20~30
#   COVERAGE_TARGET    覆盖率达标阈值，默认 0.75
MAX_GOAL_DISTANCE=${MAX_GOAL_DISTANCE:-15.0}
COVERAGE_TARGET=${COVERAGE_TARGET:-0.75}

# 出生点默认从分区 json 算：每只狗落在它负责的 box 的中心。
# （换分区必须同步，否则狗出生在 box 外 -> 地图与 box 不重叠 -> 覆盖率恒为 0。
#   之前 go2_3 就是这么变成 0% 的）
# 手动覆盖：SPAWN_POS="x1,y1;x2,y2;x3,y3"
if [ -z "${SPAWN_POS:-}" ]; then
    SPAWN_POS=$(BOX_JSON="$BOX_JSON" BOX_IDS="$BOX_IDS" python3 -c '
import json, os, sys
try:
    d = json.load(open(os.environ["BOX_JSON"]))
    ids = [int(x) for x in os.environ["BOX_IDS"].split()]
    out = []
    for bid in ids:
        b = [x for x in d["boxes"] if x["id"] == bid]
        if not b:
            continue
        c = b[0]["bbox_world"]["corners_world"]
        xs = [p[0] for p in c]
        ys = [p[1] for p in c]
        out.append("%.2f,%.2f" % ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0))
    sys.stdout.write(";".join(out))
except Exception:
    sys.stdout.write("")
')
fi
export SPAWN_POS
# Gazebo 画面很吃 CPU（实测 gzclient 约 200%），3 狗时 RTF 只有 ~0.13，
# 默认关掉 GUI 提速：需要看画面就 GUI=true bash explore_three_dogs.sh
GUI=${GUI:-false}

export GAZEBO_MODEL_PATH="/home/ubuntu/go2_target_seek_delivery_0901/KD_MODEL/models:/home/ubuntu/go2_target_seek_delivery_0901/QY_MODEL/models:$HOME/.gazebo/models"
source /home/ubuntu/go2_target_seek_delivery/setup_gpu.sh >/dev/null 2>&1
export GAZEBO_MODEL_DATABASE_URI=""
# 必须 unset：localhost-only 会把 participant 拉回 lo，且实测会重新启用 SHM 传输
unset ROS_LOCALHOST_ONLY
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profile.xml"
export ROS_DOMAIN_ID=0

for _s in /opt/ros/humble/setup.bash "$W/install/setup.bash" "$WS2/install/setup.bash"; do
    [ -f "$_s" ] && source "$_s" >/dev/null 2>&1
done

has_publisher() {
    local topic="$1"
    python3 -c "
import rclpy
from rclpy.node import Node
rclpy.init()
node = Node('chk')
pubs = []
# 新建节点后需要给 DDS 一点发现时间，否则刚 init 就查会误报 no
for _ in range(20):
    pubs = node.get_publishers_info_by_topic('$topic')
    if pubs:
        break
    rclpy.spin_once(node, timeout_sec=0.3)
print('yes' if pubs else 'no')
node.destroy_node()
rclpy.shutdown()
" 2>/dev/null
}

echo "============================================================"
echo "  三狗一键探索 (GPU渲染)"
echo "============================================================"

# [1] 清理
echo "[1/9] 清理旧实例..."
pkill -9 -f "run_rl_sim_pty" 2>/dev/null; pkill -9 -f "rl_sar/rl_sim" 2>/dev/null
pkill -9 -f "explore_simple" 2>/dev/null; sleep 1
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null; pkill -9 -f parameter_blackboard 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null; pkill -9 -f "controller_manager/spawner" 2>/dev/null
pkill -9 -f "gt_odom_bridge" 2>/dev/null
pkill -9 -f "static_transform_publisher" 2>/dev/null; pkill -9 -f "rtabmap" 2>/dev/null
pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null; pkill -9 -f rviz2 2>/dev/null
pkill -9 -f "ros2_daemon" 2>/dev/null; pkill -9 -f "_ros2_daemon" 2>/dev/null
# 上一轮的 ros2 launch 父进程和 Nav2 节点必须杀干净，否则会和新一轮抢同名节点/service
pkill -9 -f "ros2 launch" 2>/dev/null
for _n in controller_server planner_server bt_navigator behavior_server \
          smoother_server waypoint_follower velocity_smoother lifecycle_manager; do
    pkill -9 -f "$_n" 2>/dev/null
done
rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null; rm -rf ~/.ros/daemon 2>/dev/null
sleep 3
echo "      已清理"

# [2] Gazebo
echo "[2/9] 启动 forestV3 世界 (GPU渲染)..."
: > "$LOG/gazebo.log"
setsid bash -c "source /opt/ros/humble/setup.bash && source $W/install/setup.bash && \
    source /home/ubuntu/go2_target_seek_delivery/setup_gpu.sh && \
    FASTRTPS_DEFAULT_PROFILES_FILE=$SCRIPT_DIR/fastdds_profile.xml \
    ros2 launch rl_sar gazebo_world.launch.py world:=$WORLD gui:=$GUI > $LOG/gazebo.log 2>&1" &
cw=0
until [ "$(has_publisher /clock)" = "yes" ]; do
    sleep 3; cw=$((cw+3)); [ $cw -ge 150 ] && { echo "WARN: /clock 超时"; break; }
done
echo "      世界已加载"

# [3] 生成狗
echo "[3/9] 生成三只狗... (出生点: ${SPAWN_POS:-默认})"
setsid bash -c "export AMENT_PREFIX_PATH='' && source /opt/ros/humble/setup.bash && \
    source $W/install/setup.bash && \
    FASTRTPS_DEFAULT_PROFILES_FILE=$SCRIPT_DIR/fastdds_profile.xml \
    SPAWN_DOGS=3 ros2 launch rl_sar spawn_dogs.launch.py spawn_roll:=0 spawn_z:=0.5 use_camera:=true>> $LOG/gazebo.log 2>&1" &
for i in "${DOGS[@]}"; do
    tw=0
    while true; do
        result=$(has_publisher /go2_$i/joint_states 2>/dev/null)
        [ "$result" = "yes" ] && break
        sleep 2; tw=$((tw+2))
        [ $tw -ge 120 ] && { echo "WARN: go2_$i joint_states 超时"; break; }
    done
    echo "      go2_$i 已生成"
done

# 等待 controller
echo "      等待 controller..."
for i in "${DOGS[@]}"; do
    ok=0; for _ in $(seq 1 30); do
        result=$(has_publisher /go2_$i/joint_states 2>/dev/null)
        [ "$result" = "yes" ] && { ok=1; break; }; sleep 2
    done
    if [ "$ok" -eq 0 ]; then
        echo "      go2_$i 手动激活 controller..."
        python3 -c "
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ConfigureController, SwitchController
import time
rclpy.init()
node = Node('activate_$i')
for ctrl in ['joint_state_broadcaster', 'robot_joint_controller_go2_$i']:
    cli = node.create_client(ConfigureController, '/go2_$i/controller_manager/configure_controller')
    cli.wait_for_service(timeout_sec=5)
    req = ConfigureController.Request(); req.name = ctrl
    cli.call_async(req); rclpy.spin_until_future_complete(node, rclpy.Future(), timeout_sec=5)
    time.sleep(1)
    cli2 = node.create_client(SwitchController, '/go2_$i/controller_manager/switch_controller')
    cli2.wait_for_service(timeout_sec=5)
    req2 = SwitchController.Request(); req2.activate_controllers = [ctrl]; req2.strictness = 2
    cli2.call_async(req2); rclpy.spin_until_future_complete(node, rclpy.Future(), timeout_sec=5)
    time.sleep(1)
node.destroy_node(); rclpy.shutdown()
" 2>/dev/null
        sleep 3
        result=$(has_publisher /go2_$i/joint_states 2>/dev/null)
        [ "$result" = "yes" ] && ok=1
    fi
    [ "$ok" -eq 1 ] && echo "      go2_$i controller OK" || echo "      !! go2_$i controller 未检测到"
done

# [4] RL 策略
echo "[4/9] 启动 RL 策略进程..."
for i in "${DOGS[@]}"; do
    : > "$LOG/rlsim_$i.log"
    if [ ! -p "$LOG/keys_$i" ]; then rm -f "$LOG/keys_$i"; mkfifo "$LOG/keys_$i"; fi
    setsid python3 "$PTY" $i >/dev/null 2>&1 &
    tw=0; until grep -qa "RL_Sim start" "$LOG/rlsim_$i.log" 2>/dev/null; do
        sleep 2; tw=$((tw+2)); [ $tw -ge 90 ] && { echo "WARN: go2_$i rl_sim 超时"; break; }
    done; echo "      rl_sim(go2_$i) 已就绪"
done

# [5] 初始化狗 R→0→1→N
echo "[5/9] 逐狗初始化 (R→0→1→N)..."
for i in "${DOGS[@]}"; do
    echo "  [go2_$i] R 重置..."
    printf 'R' > "$LOG/keys_$i"; sleep 4
    echo "  [go2_$i] 0 起身..."
    printf '0' > "$LOG/keys_$i"; sleep 6
    for _ in $(seq 1 15); do
        tail -c 2000 "$LOG/rlsim_$i.log" 2>/dev/null | grep -qa "RLFSMStateRLLocomotion" && break; sleep 1
    done
    echo "  [go2_$i] 1 策略..."
    printf '1' > "$LOG/keys_$i"; sleep 5
    tail -c 2000 "$LOG/rlsim_$i.log" 2>/dev/null | grep -qa "RLFSMStateRLLocomotion" || { printf '1' > "$LOG/keys_$i"; sleep 4; }
    echo "  [go2_$i] N 导航..."
    for _ in $(seq 1 10); do
        tail -c 2000 "$LOG/rlsim_$i.log" 2>/dev/null | grep -qa "RLFSMStateRLLocomotion" && break; sleep 1
    done
    printf 'N' > "$LOG/keys_$i"; sleep 3
    # N 是切换键 (rl_sdk.cpp)，必须按日志里最后一次状态判断是否真的 ON，否则会二次切换回 OFF
    nav_state() { grep -ao "Navigation mode: \(ON\|OFF\)" "$LOG/rlsim_$i.log" 2>/dev/null | tail -1; }
    for _ in 1 2; do
        [ "$(nav_state)" = "Navigation mode: ON" ] && break
        printf 'N' > "$LOG/keys_$i"; sleep 3
    done
    printf ' ' > "$LOG/keys_$i"
    echo "  [go2_$i] 初始化完成"
done

# [6] odom 桥接
echo "[6/9] 启动 odom 桥接 + static TF..."
for i in "${DOGS[@]}"; do
    setsid python3 "$W/scripts/gt_odom_bridge.py" --ros-args -p namespace:=go2_$i > "$LOG/gt_odom_bridge_$i.log" 2>&1 < /dev/null &
    setsid ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 go2_$i/base_link go2_$i/base >/dev/null 2>&1 < /dev/null &
done; sleep 3; echo "      odom 桥接已启动"

# [7] Nav2（交错启动）
echo "[7/9] 启动 Nav2/RTAB-Map（每只狗间隔 25s）..."
for i in "${DOGS[@]}"; do
    setsid bash -c "source /opt/ros/humble/setup.bash && source $W/install/setup.bash && \
        source $WS2/install/setup.bash && export DELIVERY_ROOT=/home/ubuntu/go2_target_seek_delivery && \
        FASTRTPS_DEFAULT_PROFILES_FILE=$SCRIPT_DIR/fastdds_profile.xml \
        ros2 launch go2_mapping_nav go2_${i}_mapping_nav.launch.py \
            use_sim_time:=true use_merged_map:=false use_rviz:=false \
            delete_db_on_start:=true > $LOG/mapping_nav_$i.log 2>&1" &
    echo "      go2_$i Nav2 已启动，等待 25s..."; sleep 25
done
echo "      三狗建图导航栈已启动"

# [7.3] 手动激活 Nav2 lifecycle
echo "[7.3/9] 手动激活 Nav2 lifecycle..."
python3 -c "
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
import time

rclpy.init()
node = Node('nav2_activate')

nav2_nodes = ['controller_server', 'planner_server', 'bt_navigator',
              'behavior_server', 'smoother_server', 'waypoint_follower', 'velocity_smoother']
costmap_nodes = ['global_costmap/global_costmap', 'local_costmap/local_costmap']

for i in [1,2,3]:
    ns = f'/go2_{i}'
    for n in costmap_nodes + nav2_nodes:
        full = f'{ns}/{n}'
        try:
            get_cli = node.create_client(GetState, f'{full}/get_state')
            if get_cli.wait_for_service(timeout_sec=2):
                req = GetState.Request()
                fut = get_cli.call_async(req)
                rclpy.spin_until_future_complete(node, fut, timeout_sec=2)
                state = fut.result().current_state.label
                if state == 'active':
                    continue
                if state in ('unconfigured', 'inactive'):
                    cs_cli = node.create_client(ChangeState, f'{full}/change_state')
                    if cs_cli.wait_for_service(timeout_sec=2):
                        # 必须等真正的 future 完成；传一个空 Future 等于没等，
                        # 会随机漏激活(漏掉 bt_navigator -> 所有 goal 被 reject)
                        if state == 'unconfigured':
                            req = ChangeState.Request()
                            req.transition.id = 1  # configure
                            fut = cs_cli.call_async(req)
                            rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
                            time.sleep(0.5)
                        req = ChangeState.Request()
                        req.transition.id = 3  # activate
                        fut = cs_cli.call_async(req)
                        rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
                        time.sleep(0.5)
                        # 校验：没活就再试一次
                        for _retry in range(2):
                            fut = get_cli.call_async(GetState.Request())
                            rclpy.spin_until_future_complete(node, fut, timeout_sec=3)
                            try:
                                st2 = fut.result().current_state.label
                            except Exception:
                                st2 = '?'
                            if st2 == 'active':
                                break
                            req = ChangeState.Request()
                            req.transition.id = 3
                            fut = cs_cli.call_async(req)
                            rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
                            time.sleep(0.5)
        except: pass
    print(f'go2_{i} Nav2 lifecycle done')

node.destroy_node()
rclpy.shutdown()
" 2>/dev/null

# [7.5] 等待 scan 和 map（用 Python 检测 publisher）
echo "[7.5/9] 等待 scan 和 map..."
for i in "${DOGS[@]}"; do
    tw=0
    while true; do
        result=$(has_publisher /go2_$i/scan 2>/dev/null)
        [ "$result" = "yes" ] && break
        sleep 2; tw=$((tw+2))
        [ $tw -ge 90 ] && { echo "WARN: go2_$i scan 超时"; break; }
    done
    tw=0
    while true; do
        result=$(has_publisher /go2_$i/map 2>/dev/null)
        [ "$result" = "yes" ] && break
        sleep 2; tw=$((tw+2))
        [ $tw -ge 90 ] && { echo "WARN: go2_$i map 超时"; break; }
    done
    echo "      go2_$i scan+map 就绪"
done

# [8] 验证
echo "[8/9] 验证系统状态..."
sleep 5
for i in "${DOGS[@]}"; do
    NAV=$(tail -c 500 "$LOG/rlsim_$i.log" 2>/dev/null | grep -c "Navigation mode: ON")
    SCAN=$(has_publisher /go2_$i/scan 2>/dev/null)
    MAP=$(has_publisher /go2_$i/map 2>/dev/null)
    if [ "$NAV" -ge 1 ] && [ "$SCAN" = "yes" ] && [ "$MAP" = "yes" ]; then
        echo "      go2_$i: OK"
    else
        echo "      go2_$i: FAIL nav=$NAV scan=$SCAN map=$MAP"
    fi
done

# [9] 启动探索
echo "[9/9] 启动三狗探索..."
cd "$SCRIPT_DIR"
read -r -a _BOX_ARR <<< "$BOX_IDS"
for i in "${DOGS[@]}"; do
    BOX_ID=${_BOX_ARR[$((i - 1))]:-$i}
    setsid bash -c "cd '$SCRIPT_DIR' && \
        FASTRTPS_DEFAULT_PROFILES_FILE='$SCRIPT_DIR/fastdds_profile.xml' \
        exec python3 explore_simple.py \
        --box-json '$BOX_JSON' \
        --box-id $BOX_ID --robot-name go2_$i \
        --action-server /go2_$i/navigate_to_pose --map-topic /go2_$i/map \
        --save-prefix "$LOG/explored_box_${BOX_ID}_go2_$i" \
        --max-goal-distance $MAX_GOAL_DISTANCE --coverage-target $COVERAGE_TARGET" \
        > "$LOG/explore_$i.log" 2>&1 < /dev/null &
    echo "      go2_$i 探索已启动 (Box $BOX_ID)"
    sleep 2
done

echo ""
echo "============================================================"
echo "  三狗探索已全部启动!"
echo "============================================================"

# 监控
while true; do
    sleep 30
    echo "--- $(date '+%H:%M:%S') ---"
    for i in "${DOGS[@]}"; do
        CELL=$(grep -oP '选块#\K[0-9]+' "$LOG/explore_$i.log" 2>/dev/null | tail -1)
        PLAN=$(grep -oP '划分\K[0-9]+' "$LOG/explore_$i.log" 2>/dev/null | tail -1)
        CELL_COV=$(grep -oP '子区域: \K[0-9.]+' "$LOG/explore_$i.log" 2>/dev/null | tail -1)
        OVERALL=$(grep -oP '整体: \K[0-9.]+' "$LOG/explore_$i.log" 2>/dev/null | tail -1)
        echo "  go2_$i: 第${CELL:-?}次选块(当前划分${PLAN:-?}块) 子区域=${CELL_COV:-N/A}% 整体=${OVERALL:-N/A}%"
    done
    DONE=0
    for i in "${DOGS[@]}"; do
        tail -5 "$LOG/explore_$i.log" 2>/dev/null | grep -q "Exploration finished\|Too many failures\|Coverage.*95" && DONE=$((DONE+1))
    done
    [ "$DONE" -ge 3 ] && { echo "所有探索已完成!"; break; }
done
