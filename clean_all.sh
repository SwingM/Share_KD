#!/usr/bin/env bash
# clean_all.sh — 杀掉所有相关进程 + 清理共享内存
echo "清理旧实例..."
pkill -9 -f explore_three_dogs.sh
pkill -9 -f "ros2 launch"
pkill -9 -f explore_simple.py
pkill -9 -f gt_odom_bridge

pkill -9 -f "run_rl_sim_pty" 2>/dev/null
pkill -9 -f "rl_sar/rl_sim" 2>/dev/null
pkill -9 -f "ros2 launch rl_sar" 2>/dev/null
pkill -9 -f "explore_box" 2>/dev/null
pkill -9 -f "explore_region" 2>/dev/null
pkill -9 -f "explore.py" 2>/dev/null
pkill -9 -f "box_explorer" 2>/dev/null
sleep 1
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f parameter_blackboard 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null
pkill -9 -f joy_node 2>/dev/null
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
pkill -9 -f "map_saver" 2>/dev/null
pkill -9 -f "map_server" 2>/dev/null
pkill -9 -f "lifecycle_manager" 2>/dev/null
pkill -9 -f "velocity_smoother" 2>/dev/null
pkill -9 -f "smoother_server" 2>/dev/null
pkill -9 -f "waypoint_follower" 2>/dev/null
rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null
rm -rf ~/.ros/daemon 2>/dev/null
echo "清理完成"
