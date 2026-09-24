#!/usr/bin/env python3
"""
探索 shadow_building_boxes.json 中指定区域的未知区域
参考 explore_region.py 实现，支持从JSON读取边界

用法:
  python3 explore_box.py --box-id 8 --robot-name go2_1
  python3 explore_box.py --box-id 6 --robot-name go2_1  # 更大的区域
"""

import rclpy
import sys
import math
import time
import json
import argparse
import cv2
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

STEP = 5.0
FRONTIER_SEARCH_RADIUS = 6.0
FRONTIER_VICINITY = 1.5
GAMMA_1 = 0.4
GAMMA_2 = 0.2
GAMMA_3 = 0.4
FALL_HEIGHT_THRESHOLD = -10.0  # Disabled: odom reports z=0.0 even when robot is fine
FALL_CONSECUTIVE_MAX = 999
STUCK_TIMEOUT = 10.0
HEALTH_CHECK_INTERVAL = 2.0


class BoxExplorer(Node):
    def __init__(self, robot_name, box_id, box_data, action_server, map_topic, save_prefix):
        super().__init__(f'box_explorer_{robot_name}')
        self._robot_name = robot_name
        self._box_id = box_id
        self._action_server = action_server
        self._map_topic = map_topic
        self._save_prefix = save_prefix

        corners = box_data['bbox_world']['corners_world']
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        self.BOUND_X_MIN = min(xs)
        self.BOUND_X_MAX = max(xs)
        self.BOUND_Y_MIN = min(ys)
        self.BOUND_Y_MAX = max(ys)

        self.get_logger().info(
            f'Box {box_id} bounds: x[{self.BOUND_X_MIN:.2f}, {self.BOUND_X_MAX:.2f}] '
            f'y[{self.BOUND_Y_MIN:.2f}, {self.BOUND_Y_MAX:.2f}]')

        self._client = ActionClient(self, NavigateToPose, action_server)
        self._goal_done = False
        self._last_status = None

        self._map = None
        self._map_ready = False
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_cb, map_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frontiers = []
        self._best_frontier = None
        self._fall_count = 0
        self._prev_dist = None
        self._current_goal = None
        self._prev_yaw_error = None
        self._stuck_start = None
        self._health_timer = self.create_timer(HEALTH_CHECK_INTERVAL, self.health_check)
        self._retry_count = 0
        self._last_goal = None
        self._max_retries = 5
        self._global_stuck_count = 0
        self._max_global_stuck = 8
        self._skipped_goals = set()
        self._rejected = False
        self._nav2_aborted = False

        self._frontier_pub = self.create_publisher(
            Marker, f'/box_explorer_{robot_name}/frontiers', 1)
        self._best_pub = self.create_publisher(
            Marker, f'/box_explorer_{robot_name}/best_frontier', 1)
        self._region_pub = self.create_publisher(
            Marker, f'/box_explorer_{robot_name}/region', 1)
        self._cmd_vel_pub = self.create_publisher(
            Twist, f'/{robot_name}/cmd_vel', 1)

        self.get_logger().info(f'BoxExplorer {robot_name} initialized for box {box_id}')

    def map_cb(self, msg):
        self._map = msg
        if not self._map_ready:
            self._map_ready = True
            self.get_logger().info('Map received')

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                f'{self._robot_name}/map', f'{self._robot_name}/base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def coord_to_index(self, x, y):
        m = self._map
        col = int((x - m.info.origin.position.x) / m.info.resolution)
        row = int((y - m.info.origin.position.y) / m.info.resolution)
        return max(0, min(col, m.info.width - 1)), max(0, min(row, m.info.height - 1))

    def index_to_coord(self, col, row):
        m = self._map
        x = (col + 0.5) * m.info.resolution + m.info.origin.position.x
        y = (row + 0.5) * m.info.resolution + m.info.origin.position.y
        return x, y

    def get_cost(self, x, y):
        if not self._map_ready:
            return -1
        col, row = self.coord_to_index(x, y)
        idx = row * self._map.info.width + col
        return self._map.data[idx]

    def find_free_point(self, wx, wy, radius=2.0, step=0.2):
        if self.get_cost(wx, wy) < 50:
            return (wx, wy)
        r = step
        while r <= radius:
            n = max(8, int(2 * math.pi * r / step))
            for i in range(n):
                theta = 2.0 * math.pi * i / n
                cx = wx + r * math.cos(theta)
                cy = wy + r * math.sin(theta)
                if 0 <= self.get_cost(cx, cy) < 50:
                    return (cx, cy)
            r += step
        return (wx, wy)

    def in_bounds(self, x, y):
        return self.BOUND_X_MIN <= x <= self.BOUND_X_MAX and self.BOUND_Y_MIN <= y <= self.BOUND_Y_MAX

    def detect_frontiers(self, robot_pos, search_range=FRONTIER_SEARCH_RADIUS):
        if not self._map_ready:
            return []

        m = self._map
        x0, y0 = self.coord_to_index(robot_pos[0] - search_range,
                                     robot_pos[1] - search_range)
        x1, y1 = self.coord_to_index(robot_pos[0] + search_range,
                                     robot_pos[1] + search_range)
        w, h = x1 - x0, y1 - y0
        if w <= 2 or h <= 2:
            return []

        free_map = np.zeros((h, w), dtype=np.uint8)
        for r in range(h):
            row_data = m.data[(y0 + r) * m.info.width + x0: (y0 + r) * m.info.width + x0 + w]
            free_map[r, :] = (np.array(row_data) == 0).astype(np.uint8)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilated = cv2.dilate(free_map, kernel, iterations=1)
        border = dilated & ~free_map

        contours, _ = cv2.findContours(border, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pts = []
        for cnt in contours:
            if len(cnt) < 3:
                continue
            M = cv2.moments(cnt)
            if M['m00'] <= 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            wx = (x0 + cx) * m.info.resolution + m.info.origin.position.x
            wy = (y0 + cy) * m.info.resolution + m.info.origin.position.y
            if self.in_bounds(wx, wy):
                pts.append((wx, wy))

        return pts

    def merge_frontiers(self, new_pts):
        for pt in new_pts:
            dup = any(math.hypot(pt[0] - f[0], pt[1] - f[1]) < FRONTIER_VICINITY
                      for f in self.frontiers)
            if not dup:
                self.frontiers.append(pt)

    def clean_frontiers(self, robot_pos):
        keep = []
        for f in self.frontiers:
            d = math.hypot(f[0] - robot_pos[0], f[1] - robot_pos[1])
            if d < 1.5:
                continue
            c = self.get_cost(f[0], f[1])
            if c >= 50:
                continue
            keep.append(f)
        self.frontiers = keep

    def score_frontier(self, f, robot_pos, robot_yaw):
        dx = f[0] - robot_pos[0]
        dy = f[1] - robot_pos[1]
        h1 = math.hypot(dx, dy)

        # Heavily penalize far-away frontiers (prefer close ones)
        if h1 > 10.0:
            return 1000.0  # Too far

        # Check if path to frontier is mostly free (simple straight-line check)
        steps = max(int(h1 / 0.5), 1)
        blocked = 0
        for i in range(1, steps):
            t = i / steps
            cx = robot_pos[0] + dx * t
            cy = robot_pos[1] + dy * t
            cost = self.get_cost(cx, cy)
            if cost > 50:  # Occupied or unknown
                blocked += 1
        if blocked > steps * 0.3:  # More than 30% blocked
            return 1000.0

        target_angle = math.atan2(dy, dx)
        diff = abs(target_angle - robot_yaw)
        diff = min(diff, 2 * math.pi - diff)
        h2 = diff / math.pi

        col, row = self.coord_to_index(f[0], f[1])
        rp = int(2.0 / self._map.info.resolution)
        unknown = total = 0
        for dr in range(-rp, rp + 1):
            for dc in range(-rp, rp + 1):
                cr, cc = row + dr, col + dc
                if 0 <= cr < self._map.info.height and 0 <= cc < self._map.info.width:
                    total += 1
                    if self._map.data[cr * self._map.info.width + cc] == -1:
                        unknown += 1
        h3 = unknown / max(total, 1)

        box_cx = (self.BOUND_X_MIN + self.BOUND_X_MAX) / 2.0
        box_cy = (self.BOUND_Y_MIN + self.BOUND_Y_MAX) / 2.0
        dist_to_box = math.hypot(f[0] - box_cx, f[1] - box_cy)
        h4 = dist_to_box / 50.0

        return GAMMA_1 * (h1 / 50.0) + GAMMA_2 * h2 - GAMMA_3 * h3 + 0.3 * h4

    def coverage_ratio(self):
        if not self._map_ready:
            return 0.0
        x0, y0 = self.coord_to_index(self.BOUND_X_MIN, self.BOUND_Y_MIN)
        x1, y1 = self.coord_to_index(self.BOUND_X_MAX, self.BOUND_Y_MAX)
        known = total = 0
        for row in range(y0, y1 + 1):
            for col in range(x0, x1 + 1):
                if 0 <= row < self._map.info.height and 0 <= col < self._map.info.width:
                    total += 1
                    if self._map.data[row * self._map.info.width + col] >= 0:
                        known += 1
        return known / max(total, 1)

    def publish_frontiers(self):
        m = Marker()
        m.header.frame_id = f'{self._robot_name}/map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'frontiers'
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.a = 0.8
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.pose.orientation.w = 1.0
        for f in self.frontiers:
            p = Point(x=f[0], y=f[1], z=0.3)
            m.points.append(p)
        self._frontier_pub.publish(m)

    def publish_best_frontier(self):
        m = Marker()
        m.header.frame_id = f'{self._robot_name}/map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'best_frontier'
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.8
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.pose.orientation.w = 1.0
        if self._best_frontier:
            m.pose.position.x = self._best_frontier[0]
            m.pose.position.y = self._best_frontier[1]
            m.pose.position.z = 0.3
        self._best_pub.publish(m)

    def publish_region(self):
        m = Marker()
        m.header.frame_id = f'{self._robot_name}/map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'region'
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.2
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.pose.orientation.w = 1.0
        corners = [
            (self.BOUND_X_MIN, self.BOUND_Y_MIN),
            (self.BOUND_X_MAX, self.BOUND_Y_MIN),
            (self.BOUND_X_MAX, self.BOUND_Y_MAX),
            (self.BOUND_X_MIN, self.BOUND_Y_MAX),
            (self.BOUND_X_MIN, self.BOUND_Y_MIN),
        ]
        for x, y in corners:
            m.points.append(Point(x=x, y=y, z=0.0))
        self._region_pub.publish(m)

    def publish_visualization(self):
        self.publish_frontiers()
        self.publish_best_frontier()
        self.publish_region()

    def _cancel_goal(self):
        if hasattr(self, '_send_goal_future') and self._send_goal_future:
            gh = self._send_goal_future.result()
            if gh and gh.accepted:
                gh.cancel_goal_async()

    def _get_yaw_error(self):
        if self._current_goal is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                f'{self._robot_name}/map', f'{self._robot_name}/base_link', rclpy.time.Time())
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            gx, gy = self._current_goal
            target_yaw = math.atan2(gy - t.transform.translation.y,
                                    gx - t.transform.translation.x)
            diff = target_yaw - yaw
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return abs(diff)
        except Exception:
            return None

    def health_check(self):
        if self._goal_done:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                f'{self._robot_name}/map', f'{self._robot_name}/base_link', rclpy.time.Time())
            z = t.transform.translation.z
            if z < FALL_HEIGHT_THRESHOLD:
                self._fall_count += 1
                self.get_logger().warn(f'FALL DETECTED (z={z:.3f}, count={self._fall_count})')
                if self._fall_count >= FALL_CONSECUTIVE_MAX:
                    self.get_logger().error('Robot fell! Aborting goal.')
                    self._cancel_goal()
                    self._goal_done = True
                    self._last_status = -2
            else:
                self._fall_count = 0

            ye = self._get_yaw_error()
            if ye is not None and self._prev_yaw_error is not None:
                if abs(ye - self._prev_yaw_error) > 0.05:
                    self._stuck_start = None
            self._prev_yaw_error = ye
        except Exception:
            pass

    def send_goal(self, x, y, yaw=0.0):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = f'{self._robot_name}/map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        half = yaw * 0.5
        g.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(f'Action server {self._action_server} not available!')
            return False

        self._goal_done = False
        self._last_status = None
        self._fall_count = 0
        self._prev_dist = None
        self._current_goal = (x, y)
        self._prev_yaw_error = None
        self._stuck_start = None
        self._rejected = False
        self._nav2_aborted = False
        self._goal_start_time = time.time()
        self._send_goal_future = self._client.send_goal_async(g, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.response_cb)

        while rclpy.ok() and not self._goal_done:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._rejected:
            self.get_logger().info('Goal was rejected, waiting 2s before retry...')
            time.sleep(2.0)
            return False

        return self._last_status == 4

    def send_step_goal(self, tx, ty):
        pose = self.get_robot_pose()
        if pose is None:
            return False
        cx, cy = pose
        dx, dy = tx - cx, ty - cy
        d = math.hypot(dx, dy)
        if d <= 0.8:
            return True
        t = min(STEP / d, 1.0)
        wx, wy = self.find_free_point(cx + dx * t, cy + dy * t)
        return self.send_goal(wx, wy)

    def response_cb(self, future):
        h = future.result()
        if not h.accepted:
            self.get_logger().warn('Goal rejected! Nav2 may not be ready.')
            self._goal_done = True
            self._last_status = -1
            self._rejected = True  # Mark as rejected, not stuck
            return
        self._rejected = False
        self.get_logger().info('Goal accepted')
        self._get_result_future = h.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, msg):
        d = msg.feedback.distance_remaining
        if d is not None:
            # Skip initial feedback (Nav2 may report 0.00m before robot moves)
            elapsed = time.time() - self._goal_start_time
            if d <= 0.8 and elapsed > 3.0:
                self.get_logger().info(f'Close enough ({d:.2f}m), finishing goal.')
                self._cancel_goal()
                self._goal_done = True
                self._last_status = 4
                return
            if self._prev_dist is not None and abs(d - self._prev_dist) < 0.05:
                if self._stuck_start is None:
                    self._stuck_start = time.time()
                elif time.time() - self._stuck_start > STUCK_TIMEOUT:
                    self.get_logger().warn(f'STUCK! No progress for {STUCK_TIMEOUT}s, smart recovery...')
                    self._cancel_goal()
                    self._goal_done = True
                    self._last_status = -3
                    self.smart_recovery()
                    return
            else:
                self._stuck_start = None
            self._prev_dist = d

    def smart_recovery(self):
        """Smart recovery: try multiple directions to get unstuck."""
        pose = self.get_robot_pose()
        if pose is None:
            return
        self.get_logger().info('Attempting smart recovery: back+turn combinations...')

        # Strategy 1: Back up while turning
        for turn_dir in [1.0, -1.0]:  # Try both directions
            msg = Twist()
            msg.linear.x = -0.4
            msg.angular.z = turn_dir * 1.5
            for _ in range(20):  # 2 seconds
                self._cmd_vel_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.1)
            self._cmd_vel_pub.publish(Twist())
            time.sleep(0.3)
            # Check if we moved
            new_pose = self.get_robot_pose()
            if new_pose and math.hypot(new_pose[0] - pose[0], new_pose[1] - pose[1]) > 0.2:
                self.get_logger().info(f'Recovery OK: moved {math.hypot(new_pose[0] - pose[0], new_pose[1] - pose[1]):.2f}m')
                return

        # Strategy 2: Turn in place, then move forward
        for turn_dir in [1.0, -1.0]:
            msg = Twist()
            msg.angular.z = turn_dir * 2.0
            for _ in range(25):  # 2.5 seconds
                self._cmd_vel_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.1)
            self._cmd_vel_pub.publish(Twist())
            time.sleep(0.2)
            # Now try forward
            msg = Twist()
            msg.linear.x = 0.4
            for _ in range(15):  # 1.5 seconds
                self._cmd_vel_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.1)
            self._cmd_vel_pub.publish(Twist())
            time.sleep(0.3)
            new_pose = self.get_robot_pose()
            if new_pose and math.hypot(new_pose[0] - pose[0], new_pose[1] - pose[1]) > 0.2:
                self.get_logger().info(f'Recovery OK: turned then moved')
                return

        # Strategy 3: Sideways (if possible with diff drive, just spin more)
        msg = Twist()
        msg.angular.z = 3.0
        for _ in range(40):  # 4 seconds full spin
            self._cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().warn('Recovery complete (robot may still be stuck)')

    def result_cb(self, future):
        r = future.result()
        self._last_status = r.status
        s = r.status
        if s == 4:
            self.get_logger().info('  SUCCEEDED!')
            self._global_stuck_count = 0
            self._nav2_aborted = False
        elif s == 3:
            self.get_logger().info('  Canceled.')
            self._nav2_aborted = False
        elif s in (5, 6):
            self.get_logger().info(f'  Nav2 aborted (status:{s})')
            self._nav2_aborted = True
        else:
            self.get_logger().info(f'  Status: {s}')
            self._nav2_aborted = True
        self._goal_done = True

    def get_robot_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                f'{self._robot_name}/map', f'{self._robot_name}/base_link', rclpy.time.Time())
            q = t.transform.rotation
            return math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        except Exception:
            return 0.0

    def save_map(self):
        self.get_logger().info('Saving RTAB-Map database...')
        robot_idx = self._robot_name.split('_')[-1]
        save_srv = f'/rtabmap_{robot_idx}/save'
        client = self.create_client(Empty, save_srv)
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            if future.result() is not None:
                self.get_logger().info(f'RTAB-Map database saved ({save_srv})')
            else:
                self.get_logger().warn('Save service call failed')
        else:
            self.get_logger().warn(f'{save_srv} not available, saving via CLI...')
            import subprocess
            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', self._save_prefix,
                '--ros-args', '-p', f'map_topic:={self._map_topic}',
            ], check=False)

    def run(self):
        self.get_logger().info(f'Waiting for {self._map_topic} (up to 180s)...')
        start = time.time()
        while not self._map_ready and time.time() - start < 180:
            try:
                rclpy.spin_once(self, timeout_sec=0.5)
            except Exception:
                self.get_logger().warn('ROS context lost during map wait')
                time.sleep(1)
                continue
            elapsed = time.time() - start
            if int(elapsed) % 30 == 0 and elapsed > 0:
                self.get_logger().info(f'  Still waiting for map... ({elapsed:.0f}s)')
        if not self._map_ready:
            self.get_logger().error('No map received after 180s')
            return False

        self.get_logger().info(f'Waiting for action server {self._action_server}...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Action server unavailable')
            return False

        self.publish_region()
        self.get_logger().info(f'Box {self._box_id} region marker published to RViz')

        self.get_logger().info(
            f'Starting frontier exploration towards box {self._box_id}...')

        idle = 0

        while rclpy.ok():
            try:
                pose = self.get_robot_pose()
            except Exception:
                self.get_logger().warn('ROS context lost, exiting...')
                break
            if pose is None:
                try:
                    rclpy.spin_once(self, timeout_sec=0.5)
                except Exception:
                    break
                continue

            try:
                yaw = self.get_robot_yaw()
                cov = self.coverage_ratio()

                pts = self.detect_frontiers(pose)
                if pts:
                    self.merge_frontiers(pts)

                self.clean_frontiers(pose)
            except Exception:
                self.get_logger().warn('ROS context lost during processing')
                break

            cov_pct = cov * 100

            if not self.frontiers:
                pts = self.detect_frontiers(pose, 20.0)
                if pts:
                    self.merge_frontiers(pts)

            if not self.frontiers:
                # No frontiers - check if we should stop or move to center
                box_cx = (self.BOUND_X_MIN + self.BOUND_X_MAX) / 2.0
                box_cy = (self.BOUND_Y_MIN + self.BOUND_Y_MAX) / 2.0
                dist_to_center = math.hypot(box_cx - pose[0], box_cy - pose[1])

                if dist_to_center < 2.0:
                    # Near center, exploration complete
                    self.get_logger().info(
                        f'Exploration complete! Coverage: {cov_pct:.1f}% '
                        f'(box {self._box_id}: {self.BOUND_X_MIN:.1f}-{self.BOUND_X_MAX:.1f}, '
                        f'{self.BOUND_Y_MIN:.1f}-{self.BOUND_Y_MAX:.1f})')
                    break

                # Try moving toward center via free space
                dx = box_cx - pose[0]
                dy = box_cy - pose[1]
                dist = math.hypot(dx, dy)
                step = min(STEP, dist)
                tx = pose[0] + dx / dist * step
                ty = pose[1] + dy / dist * step
                wx, wy = self.find_free_point(tx, ty, radius=3.0)
                if (wx, wy) != (tx, ty):
                    # Can't find free path to center, try exploration with larger radius
                    self.get_logger().warn(
                        f'No frontiers and no free path to center, trying wider search...')
                    pts = self.detect_frontiers(pose, 30.0)
                    if pts:
                        self.merge_frontiers(pts)
                    else:
                        self.get_logger().info(
                            f'No accessible frontiers. Coverage: {cov_pct:.1f}%')
                        break
                else:
                    self.frontiers.append((wx, wy))
                    self.get_logger().info(
                        f'Heading to box center: ({wx:.1f}, {wy:.1f}) | Coverage: {cov_pct:.1f}%')
            idle = 0

            # Filter out blacklisted goals
            available = [f for f in self.frontiers
                         if (round(f[0], 1), round(f[1], 1)) not in self._skipped_goals]
            if not available:
                self.get_logger().warn(
                    f'All {len(self.frontiers)} frontiers blacklisted. '
                    f'Coverage: {cov_pct:.1f}%. Stopping.')
                break

            best = min(available,
                       key=lambda f: self.score_frontier(f, pose, yaw))
            self._best_frontier = best

            self.get_logger().info(
                f'Goal: ({best[0]:.1f}, {best[1]:.1f}) | Coverage: {cov_pct:.1f}%')

            self.publish_visualization()

            # Track retries for same goal
            goal_key = (round(best[0], 1), round(best[1], 1))
            if goal_key == self._last_goal:
                self._retry_count += 1
            else:
                self._retry_count = 0
                self._last_goal = goal_key

            if self._retry_count >= self._max_retries:
                self._global_stuck_count += 1
                self.get_logger().warn(
                    f'Goal {goal_key} failed {self._max_retries} times, '
                    f'blacklisting and skipping... (stuck: {self._global_stuck_count}/{self._max_global_stuck})')
                self._skipped_goals.add(goal_key)
                if best in self.frontiers:
                    self.frontiers.remove(best)
                self._last_goal = None
                self._retry_count = 0

                if self._global_stuck_count >= self._max_global_stuck:
                    self.get_logger().info(
                        f'Too many stuck events ({self._global_stuck_count}). '
                        f'Final coverage: {cov_pct:.1f}%')
                    break
                continue

            if not self.send_step_goal(best[0], best[1]):
                if self._rejected:
                    self.get_logger().info(
                        f'Goal {goal_key} rejected by Nav2, retrying next iteration')
                elif self._nav2_aborted:
                    # Nav2 aborted - goal unreachable, blacklist it
                    self.get_logger().warn(
                        f'Goal {goal_key} aborted by Nav2 (unreachable), blacklisting')
                    self._skipped_goals.add(goal_key)
                    if best in self.frontiers:
                        self.frontiers.remove(best)
                else:
                    self._global_stuck_count += 1
                    self.get_logger().warn(
                        f'Goal {goal_key} failed, '
                        f'stuck: {self._global_stuck_count}/{self._max_global_stuck}')
                    if self._global_stuck_count >= self._max_global_stuck:
                        self.get_logger().info(
                            f'Too many stuck events. Final coverage: {cov_pct:.1f}%')
                        break

            if cov > 0.98:
                self.get_logger().info('Coverage > 98%, exploration done!')
                break

        self._best_frontier = None
        try:
            self.publish_visualization()
        except Exception:
            pass
        self.get_logger().info('Exploration finished! Saving map...')
        self.save_map()
        return True


def main():
    parser = argparse.ArgumentParser(description='Explore unknown area in a box region')
    parser.add_argument('--box-json', required=True,
                        help='Path to shadow_building_boxes.json')
    parser.add_argument('--box-id', type=int, required=True,
                        help='Box ID to explore (1-8)')
    parser.add_argument('--robot-name', default='go2_1',
                        help='Robot name (default: go2_1)')
    parser.add_argument('--action-server', default='/navigate_to_pose',
                        help='Nav2 action server name')
    parser.add_argument('--map-topic', default='/map',
                        help='Map topic name')
    parser.add_argument('--save-prefix', default='explored_box_map',
                        help='Prefix for saved map file')
    args = parser.parse_args()

    with open(args.box_json, 'r') as f:
        data = json.load(f)

    box = None
    for b in data['boxes']:
        if b['id'] == args.box_id:
            box = b
            break

    if box is None:
        print(f'Error: Box {args.box_id} not found in {args.box_json}')
        print(f'Available box IDs: {[b["id"] for b in data["boxes"]]}')
        sys.exit(1)

    rclpy.init()
    e = BoxExplorer(
        args.robot_name, args.box_id, box,
        args.action_server, args.map_topic, args.save_prefix)
    ok = e.run()
    e.destroy_node()
    rclpy.shutdown()
    print(f'\n==== Exploration box {args.box_id} {"SUCCESS" if ok else "FAILED"} ====')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
