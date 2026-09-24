import rclpy
import sys
import math
import time
import argparse
import cv2
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from collections import deque

STEP = 6.0
FRONTIER_SEARCH_RADIUS = 8.0
FRONTIER_VICINITY = 1.5
GAMMA_1 = 0.4
GAMMA_2 = 0.2
GAMMA_3 = 0.4
FALL_HEIGHT_THRESHOLD = 0.12
FALL_CONSECUTIVE_MAX = 5
STUCK_TIMEOUT = 30.0
HEALTH_CHECK_INTERVAL = 2.0


class Explorer(Node):
    def __init__(self, robot_name, action_server, map_topic,
                 bound_x_min, bound_x_max, bound_y_min, bound_y_max,
                 save_prefix):
        super().__init__(f'explorer_{robot_name}')
        self._robot_name = robot_name
        self._action_server = action_server
        self._map_topic = map_topic
        self._save_prefix = save_prefix

        self.BOUND_X_MIN = bound_x_min
        self.BOUND_X_MAX = bound_x_max
        self.BOUND_Y_MIN = bound_y_min
        self.BOUND_Y_MAX = bound_y_max

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

        self._frontier_pub = self.create_publisher(
            Marker, f'/explorer_{robot_name}/frontiers', 1)
        self._best_pub = self.create_publisher(
            Marker, f'/explorer_{robot_name}/best_frontier', 1)
        self._region_pub = self.create_publisher(
            Marker, f'/explorer_{robot_name}/region', 1)

        self.get_logger().info(f'Explorer {robot_name} initialized')

    def map_cb(self, msg):
        self._map = msg
        if not self._map_ready:
            self._map_ready = True
            self.get_logger().info('Map received')

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{self._robot_name}/base_link', rclpy.time.Time())
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
            if c == -1 or c >= 50:
                continue
            keep.append(f)
        self.frontiers = keep

    def score_frontier(self, f, robot_pos, robot_yaw):
        dx = f[0] - robot_pos[0]
        dy = f[1] - robot_pos[1]
        h1 = math.hypot(dx, dy)

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

        return GAMMA_1 * (h1 / 50.0) + GAMMA_2 * h2 - GAMMA_3 * h3

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
        m.header.frame_id = 'map'
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
        m.header.frame_id = 'map'
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
        m.header.frame_id = 'map'
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
                'map', f'{self._robot_name}/base_link', rclpy.time.Time())
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
                'map', f'{self._robot_name}/base_link', rclpy.time.Time())
            z = t.transform.translation.z
            if z < FALL_HEIGHT_THRESHOLD:
                self._fall_count += 1
                self.get_logger().warn(f'FALL DETECTED (z={z:.3f}, count={self._fall_count})')
                if self._fall_count >= FALL_CONSECUTIVE_MAX:
                    self.get_logger().error(f'Robot fell! Aborting goal.')
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
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        half = yaw * 0.5
        g.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(f'Action server {self._action_server} not available!')
            return False

        self.get_logger().info(f'Goal: ({x:.1f}, {y:.1f})')
        self._goal_done = False
        self._last_status = None
        self._fall_count = 0
        self._prev_dist = None
        self._current_goal = (x, y)
        self._prev_yaw_error = None
        self._stuck_start = None
        self._send_goal_future = self._client.send_goal_async(g, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.response_cb)

        while rclpy.ok() and not self._goal_done:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._last_status == 4

    def send_step_goal(self, tx, ty):
        pose = self.get_robot_pose()
        if pose is None:
            return False
        cx, cy = pose
        dx, dy = tx - cx, ty - cy
        d = math.hypot(dx, dy)
        if d <= 1.0:
            return True
        t = min(STEP / d, 1.0)
        wx, wy = self.find_free_point(cx + dx * t, cy + dy * t)
        return self.send_goal(wx, wy)

    def response_cb(self, future):
        h = future.result()
        if not h.accepted:
            self.get_logger().error('Goal rejected!')
            self._goal_done = True
            self._last_status = -1
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = h.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, msg):
        d = msg.feedback.distance_remaining
        if d is not None:
            print(f'  Remaining: {d:.2f} m   ', end='\r', flush=True)
            if self._prev_dist is not None and abs(d - self._prev_dist) < 0.05:
                if self._stuck_start is None:
                    self._stuck_start = time.time()
                elif time.time() - self._stuck_start > STUCK_TIMEOUT:
                    self.get_logger().warn(f'STUCK! No progress for {STUCK_TIMEOUT}s')
                    self._cancel_goal()
                    self._goal_done = True
                    self._last_status = -3
            else:
                self._stuck_start = None
            self._prev_dist = d

    def result_cb(self, future):
        r = future.result()
        self._last_status = r.status
        s = r.status
        if s == 4:
            self.get_logger().info('  SUCCEEDED!')
        elif s == 3:
            self.get_logger().info('  Canceled.')
        else:
            self.get_logger().info(f'  Status: {s}')
        self._goal_done = True

    def get_robot_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{self._robot_name}/base_link', rclpy.time.Time())
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
                '--ros-args', '-p', 'save_map_timeout:=30',
                '-p', f'map_topic:={self._map_topic}',
            ], check=False)

    def run(self):
        self.get_logger().info(f'Waiting for {self._map_topic} (up to 180s)...')
        start = time.time()
        while not self._map_ready and time.time() - start < 180:
            rclpy.spin_once(self, timeout_sec=0.5)
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

        self.get_logger().info('Exploration started!')
        idle = 0

        while rclpy.ok():
            pose = self.get_robot_pose()
            if pose is None:
                rclpy.spin_once(self, timeout_sec=0.5)
                continue

            yaw = self.get_robot_yaw()
            cov = self.coverage_ratio()

            pts = self.detect_frontiers(pose)
            if pts:
                self.merge_frontiers(pts)

            self.clean_frontiers(pose)

            cov_pct = cov * 100
            self.get_logger().info(
                f'Frontiers: {len(self.frontiers)} | Coverage: {cov_pct:.1f}%')

            if not self.frontiers:
                pts = self.detect_frontiers(pose, 20.0)
                if pts:
                    self.merge_frontiers(pts)

            if not self.frontiers:
                idle += 1
                if idle >= 5:
                    self.get_logger().info('No more frontiers! Exploration complete.')
                    break
                self._best_frontier = None
                self.publish_visualization()
                rclpy.spin_once(self, timeout_sec=1.0)
                continue
            idle = 0

            best = min(self.frontiers,
                       key=lambda f: self.score_frontier(f, pose, yaw))
            self._best_frontier = best

            self.get_logger().info(
                f'Best frontier: ({best[0]:.1f}, {best[1]:.1f})')

            self.publish_visualization()

            if not self.send_step_goal(best[0], best[1]):
                self.frontiers.remove(best)

            if cov > 0.98:
                self.get_logger().info('Coverage > 98%, exploration done!')
                break

        self._best_frontier = None
        self.publish_visualization()
        self.get_logger().info('Exploration finished! Saving map...')
        self.save_map()
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-name', required=True)
    parser.add_argument('--action-server', default='/navigate_to_pose')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--bounds-x-min', type=float, required=True)
    parser.add_argument('--bounds-x-max', type=float, required=True)
    parser.add_argument('--bounds-y-min', type=float, required=True)
    parser.add_argument('--bounds-y-max', type=float, required=True)
    parser.add_argument('--save-prefix', default='explored_map')
    args = parser.parse_args()

    rclpy.init()
    e = Explorer(
        args.robot_name, args.action_server, args.map_topic,
        args.bounds_x_min, args.bounds_x_max,
        args.bounds_y_min, args.bounds_y_max,
        args.save_prefix)
    ok = e.run()
    e.destroy_node()
    rclpy.shutdown()
    print(f'\n==== Exploration {args.robot_name} {"SUCCESS" if ok else "FAILED"} ====')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
