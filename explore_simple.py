#!/usr/bin/env python3
"""
简化版区域探索 - 更稳健，减少卡住
"""
import rclpy
import math
import time
import json
import os
import argparse
import numpy as np
import cv2
from collections import deque
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# ---------- 探索参数 ----------
STEP = 4.0
MAX_GOAL_DISTANCE = 15.0          # 前沿搜索半径(米)，加大以配合信息增益选点
ARRIVE_TOL = 1.5
NO_FRONTIER_ROUNDS = 5            # 子区域内连续无前沿轮数即切换(3 太容易因瞬时抖动提前放弃)
MIN_GOAL_DISTANCE = 3.0
STUCK_TIMEOUT = 15.0
GOAL_TIMEOUT = 90.0
MAX_RETRIES = 3
RECOVERY_DURATION = 2.0
SUBREGION_COVERAGE_TARGET = 0.75  # 子区域覆盖率阈值 75%

# ---------- 信息增益参数 ----------
SENSOR_RADIUS = 6.0               # 扇形视野半径(米)
SENSOR_ANGLE = math.pi            # 扇形角度 180°
SENSOR_RAYS = 36                  # 扇形内射线数

# ---------- 前沿聚类参数 ----------
CLUSTER_GRID_SIZE = 4.0           # 网格聚类尺寸(米)

# ---------- 子区域划分参数 ----------
# 子区域尺寸按"米"定义，运行时按地图分辨率换算成格子数。
# 原来写死 800 格 = 800 * 0.05^2 = 2 平方米，量级错了，导致几乎每块都被二次切割。
SUBREGION_TARGET_M = 30.0         # 目标子区域边长(米)，超过则二次切割
SUBREGION_MIN_M = 2.0             # 小于此尺寸的子区域直接丢弃
NO_FRONTIER_ROUNDS_GLOBAL = 5     # 全局无前沿轮数


class SimpleExplorer(Node):
    def __init__(self, robot_name, box_id, box_data, action_server, map_topic, save_prefix,
                 max_goal_distance=15.0, coverage_target=0.95, auto_split=0.0):
        super().__init__(f'explorer_{robot_name}')
        self._robot_name = robot_name
        self._box_id = box_id
        self._save_prefix = save_prefix
        self.MAX_GOAL_DISTANCE = max_goal_distance
        self.COVERAGE_TARGET = coverage_target
        self._cell_order = 0
        self._cell_total = 1
        self._full_bounds = None

        corners = box_data['bbox_world']['corners_world']
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        self.BOUND_X_MIN = min(xs)
        self.BOUND_X_MAX = max(xs)
        self.BOUND_Y_MIN = min(ys)
        self.BOUND_Y_MAX = max(ys)

        self.get_logger().info(
            f'Box {box_id}: x[{self.BOUND_X_MIN:.1f}, {self.BOUND_X_MAX:.1f}] '
            f'y[{self.BOUND_Y_MIN:.1f}, {self.BOUND_Y_MAX:.1f}]')

        self._client = ActionClient(self, NavigateToPose, action_server)
        self._map = None
        self._map_ready = False

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._cmd_vel_pub = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 1)
        # marker 一律用 TRANSIENT_LOCAL(latch)：RViz 晚启动也能立刻收到最后一帧，
        # 否则非 latch 的话题只在发布的那一瞬间有数据，RViz 永远看不到
        latch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._region_pub = self.create_publisher(
            Marker, f'/explorer_{robot_name}/region', latch_qos)
        # 前沿点 / 当前目标，方便在 RViz 里直接看它在想什么
        self._frontier_pub = self.create_publisher(
            Marker, f'/explorer_{robot_name}/frontiers', latch_qos)
        self._goal_pub = self.create_publisher(
            Marker, f'/explorer_{robot_name}/goal', latch_qos)
        bigbox_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._bigbox_pub = self.create_publisher(
            Marker, f'/explorer_{robot_name}/bigbox', bigbox_qos)
        # 所有子区域可视化（LINE_LIST，颜色区分状态）
        self._subregions_pub = self.create_publisher(
            Marker, f'/explorer_{robot_name}/subregions', latch_qos)

        # ground truth 里程计：TF 里只有平面位姿(z=0/无侧倾)，卡住诊断需要真实姿态
        self._gt_odom = None
        self._fall_logged = False
        self.create_subscription(
            Odometry, f'/{robot_name}/odom/ground_truth', self._gt_odom_cb, 10)

        # 局部代价地图：卡住时用来判断是否被膨胀层/致命格包住
        self._costmap = None
        self.create_subscription(
            OccupancyGrid, f'/{robot_name}/local_costmap/costmap',
            self._costmap_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))

        # 仿真时钟：Gazebo 的 /clock 是 BEST_EFFORT + VOLATILE
        # 仿真时钟：Gazebo 的 /clock 是 BEST_EFFORT + VOLATILE
        # 长时间运行时 /clock 可能会停更，所以额外记录收到时刻，_now() 里做停滞外推
        self._warned_box_outside = False
        self._sim_time = None      # 最近一次仿真时间
        self._sim_wall = 0.0       # 收到该仿真时间时的墙上时刻
        self._anchor_sim = None    # 停滞时从该点按墙上时间外推
        self._anchor_wall = 0.0
        clock_qos = QoSProfile(depth=10,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Clock, '/clock', self._clock_cb, clock_qos)

        # 子区域可视化缓存 + 定时重发（RViz 后启动也能看到）
        self._last_sub = None
        self._last_sub_cur = -1
        self._last_sub_done = set()
        self._last_sub_bounds = set()
        self.create_timer(2.0, self._republish_markers)

        self._goal_done = True
        self._last_status = None
        self._prev_dist = None
        self._stuck_start = None
        self._goal_seq = 0

    def _gt_odom_cb(self, msg):
        self._gt_odom = msg

    def _costmap_cb(self, msg):
        if self._costmap is None:
            self.get_logger().info(
                f'收到局部代价地图 {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution:.3f}m（卡住诊断已就绪）')
        self._costmap = msg

    def _is_fallen(self):
        """是否翻倒：滚转超过 90° 或高度明显低于站立高度(约 0.38m)。

        实测 go2_3 翻了 110 次、卡在 42%：翻倒后系统不知道它翻了，
        还在反复发 cmd_vel 做"后退转向"自救（徒劳，而且可能越弄越翻）。
        """
        o = self._gt_odom
        if o is None:
            return False
        q = o.pose.pose.orientation
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        z = o.pose.pose.position.z
        return abs(math.degrees(roll)) > 90.0 or z < 0.18

    def _log_fall(self):
        """翻车时大声报警，并给出可以直接复制执行的翻回命令。"""
        p = self._get_pose()
        att = self._get_attitude()
        pos = f'({p[0]:.1f},{p[1]:.1f})' if p else '?'
        extra = ''
        if att:
            extra = (f'roll={att[0]:.1f}° pitch={att[1]:.1f}° z={att[3]:.2f}m')
        fix = ''
        if p and att:
            fix = (f' 翻回来: gz model -m {self._robot_name}_gazebo '
                   f'-x {p[0]:.2f} -y {p[1]:.2f} -z 0.45 -R 0 -P 0 -Y {math.radians(att[2]):.3f}')
        self.get_logger().error(
            f'[FALL] {self._robot_name} 翻倒了! 位置={pos} {extra}.{fix}')

    def _get_attitude(self):
        """取 roll/pitch/yaw/高度，用于判断是不是在爬坡/歪着/摔倒。

        优先用 /go2_N/odom/ground_truth（含完整 3D 位姿）；TF 里只有平面位姿，
        z 恒为 0、侧倾恒为 0，拿它判断会误导。
        """
        o = self._gt_odom
        if o is not None:
            p = o.pose.pose.position
            q = o.pose.pose.orientation
            roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                              1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw), p.z)
        try:
            t = self.tf_buffer.lookup_transform(
                f'{self._robot_name}/map', f'{self._robot_name}/base_link',
                rclpy.time.Time())
            q = t.transform.rotation
            roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                              1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                    t.transform.translation.z)
        except Exception:
            return None

    def _cost_at(self, x, y):
        m = self._costmap
        if m is None:
            return None
        col = int((x - m.info.origin.position.x) / m.info.resolution)
        row = int((y - m.info.origin.position.y) / m.info.resolution)
        if not (0 <= col < m.info.width and 0 <= row < m.info.height):
            return None
        return m.data[row * m.info.width + col]

    def _stuck_diag(self, goal_xy=None):
        """卡住时打印现场，不用开 GUI 也能判断原因：

        · 姿态：是不是在爬坡/歪着/摔倒（roll/pitch/z）
        · 局部代价地图：狗是不是被膨胀层或致命格包住
        · 四个方向（机器人朝向）0.5/1.0/1.5m 处的代价值
        """
        p = self._get_pose()
        g = goal_xy or getattr(self, '_goal_xy', None)
        parts = []
        if p:
            parts.append(f'位置=({p[0]:.1f},{p[1]:.1f})')
        if p and g:
            parts.append(f'目标=({g[0]:.1f},{g[1]:.1f}) 距离={math.hypot(g[0]-p[0], g[1]-p[1]):.1f}m')
        att = self._get_attitude()
        yaw = 0.0
        if att:
            yaw = math.radians(att[2])
            parts.append(f'roll={att[0]:.1f}° pitch={att[1]:.1f}° z={att[3]:.2f}m')
        m = self._costmap
        if m is None or p is None:
            parts.append(f'局部代价地图={"未收到" if m is None else "无位置"}')
        else:
            lethal = infl = free = 0
            for dx in np.arange(-1.5, 1.51, 0.1):
                for dy in np.arange(-1.5, 1.51, 0.1):
                    if dx * dx + dy * dy > 2.25:
                        continue
                    c = self._cost_at(p[0] + dx, p[1] + dy)
                    if c is None:
                        continue
                    if c >= 99:
                        lethal += 1
                    elif c > 0:
                        infl += 1
                    else:
                        free += 1
            parts.append(f'周围1.5m: 致命={lethal} 膨胀={infl} 空闲={free}')
            fwd = (math.cos(yaw), math.sin(yaw))
            left = (-math.sin(yaw), math.cos(yaw))
            seg = []
            for name, vec in (('前', fwd), ('后', (-fwd[0], -fwd[1])),
                              ('左', left), ('右', (-left[0], -left[1]))):
                cs = [self._cost_at(p[0] + vec[0] * d, p[1] + vec[1] * d)
                      for d in (0.5, 1.0, 1.5)]
                seg.append(f'{name}={cs}')
            parts.append('方向代价(0.5/1.0/1.5m) ' + ' '.join(seg))
        self.get_logger().warn('[STUCK-DIAG] ' + ' | '.join(parts))

    def _clock_cb(self, msg):
        self._sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        self._sim_wall = time.time()

    def _now(self):
        """单调时钟（秒）。正常时等于仿真时间（Gazebo RTF << 1，判超时/卡死必须用仿真时钟）；
        /clock 停更超过 5s 时按墙上时间外推，保证绝不冻死（否则 _drive 会死循环）。"""
        if self._sim_time is None:
            return time.time()
        if time.time() - self._sim_wall <= 5.0:
            self._anchor_sim = self._sim_time
            self._anchor_wall = time.time()
            return self._sim_time
        if self._anchor_sim is None:
            self._anchor_sim = self._sim_time
            self._anchor_wall = self._sim_wall
        return self._anchor_sim + (time.time() - self._anchor_wall)

    def _drive(self, linear, angular, sim_seconds):
        """按仿真时间下发一段速度指令（原来按固定循环次数，RTF 低时等于没动）。"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        end = self._now() + sim_seconds
        wall_deadline = time.time() + max(60.0, sim_seconds * 15.0)
        while rclpy.ok() and self._now() < end and time.time() < wall_deadline:
            self._cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self._cmd_vel_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)

    def _map_cb(self, msg):
        self._map = msg
        if not self._map_ready:
            self._map_ready = True
            self.get_logger().info('Map received')

    def _get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                f'{self._robot_name}/map', f'{self._robot_name}/base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _coord_to_idx(self, x, y):
        m = self._map
        col = int((x - m.info.origin.position.x) / m.info.resolution)
        row = int((y - m.info.origin.position.y) / m.info.resolution)
        return max(0, min(col, m.info.width - 1)), max(0, min(row, m.info.height - 1))

    def _get_cost(self, x, y):
        col, row = self._coord_to_idx(x, y)
        return self._map.data[row * self._map.info.width + col]

    def _is_free(self, x, y):
        c = self._get_cost(x, y)
        return c >= 0 and c < 50

    def _find_nearest_free(self, x, y, radius=2.0):
        if self._is_free(x, y):
            return (x, y)
        for r in np.arange(0.2, radius, 0.2):
            for angle in np.arange(0, 2 * math.pi, math.pi / 4):
                nx = x + r * math.cos(angle)
                ny = y + r * math.sin(angle)
                if self._is_free(nx, ny):
                    return (nx, ny)
        return (x, y)

    def _raw_idx(self, x, y):
        """不做夹取的栅格索引，可能越界（用来判断 box 是否超出地图）。"""
        m = self._map
        return (int((x - m.info.origin.position.x) / m.info.resolution),
                int((y - m.info.origin.position.y) / m.info.resolution))

    def _known_and_totals(self):
        """返回 (已知格数, box 全部格数, box∩地图 格数)。全部用 numpy，避免几百万次 Python 循环。"""
        m = self._map
        rc0, rr0 = self._raw_idx(self.BOUND_X_MIN, self.BOUND_Y_MIN)
        rc1, rr1 = self._raw_idx(self.BOUND_X_MAX, self.BOUND_Y_MAX)
        c0, c1 = min(rc0, rc1), max(rc0, rc1)
        r0, r1 = min(rr0, rr1), max(rr0, rr1)
        total_box = max(1, (c1 - c0 + 1) * (r1 - r0 + 1))
        mc0, mc1 = max(0, c0), min(m.info.width - 1, c1)
        mr0, mr1 = max(0, r0), min(m.info.height - 1, r1)
        if mc1 < mc0 or mr1 < mr0:
            return 0, total_box, 0
        arr = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        sub = arr[mr0:mr1 + 1, mc0:mc1 + 1]
        known = int(np.count_nonzero(sub >= 0))
        return known, total_box, int(sub.size)

    def _coverage(self):
        """主指标：已知格 / box 全部格子（未建图的部分算未知）。

        之前用"已知 / (已建图 ∩ box)"，地图没覆盖到的区域完全不计入分母，
        导致大 box 覆盖率虚高、而且地图包围盒一变大数字就会掉
        （实测 go2_1 显示 32% 而真实只探了 12%）。这个口径单调递增、不会虚高。
        """
        if not self._map_ready:
            return 0.0
        known, total_box, _ = self._known_and_totals()
        return known / max(total_box, 1)

    def _coverage_mapped(self):
        """旧口径：已知格 / (已建图 ∩ box)。保留作对照。"""
        if not self._map_ready:
            return 0.0
        known, _, inter = self._known_and_totals()
        return known / max(inter, 1)

    def _detect_frontiers(self, robot_pos, radius=MAX_GOAL_DISTANCE):
        m = self._map
        x0, y0 = self._coord_to_idx(robot_pos[0] - radius, robot_pos[1] - radius)
        x1, y1 = self._coord_to_idx(robot_pos[0] + radius, robot_pos[1] + radius)
        w, h = x1 - x0, y1 - y0
        if w <= 2 or h <= 2:
            return []

        # free=1, unknown=2, occupied=0
        cell_type = np.zeros((h, w), dtype=np.uint8)
        for r in range(h):
            row_data = np.array(m.data[(y0 + r) * m.info.width + x0: (y0 + r) * m.info.width + x0 + w])
            cell_type[r, :] = np.where(row_data == 0, 1, np.where(row_data == -1, 2, 0))

        # 前沿 = free 膨胀后与 unknown 的交集（只找 free→unknown 边界，忽略墙）
        free_mask = (cell_type == 1).astype(np.uint8)
        unknown_mask = (cell_type == 2).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilated_free = cv2.dilate(free_mask, kernel, iterations=1)
        frontier = dilated_free & unknown_mask

        contours, _ = cv2.findContours(frontier, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
            if self.BOUND_X_MIN - 2 <= wx <= self.BOUND_X_MAX + 2 and \
               self.BOUND_Y_MIN - 2 <= wy <= self.BOUND_Y_MAX + 2:
                dist = math.hypot(wx - robot_pos[0], wy - robot_pos[1])
                if 0.5 < dist < radius:
                    pts.append((wx, wy))
        return pts

    def _send_goal(self, x, y):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = f'{self._robot_name}/map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x
        g.pose.pose.position.y = y
        g.pose.pose.orientation.w = 1.0

        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Action server not available')
            return False

        self._goal_done = False
        self._last_status = None
        self._goal_xy = (x, y)
        self._prev_dist = None
        self._stuck_start = None
        self._goal_start_time = self._now()
        self._goal_seq += 1
        my_seq = self._goal_seq

        future = self._client.send_goal_async(g, feedback_callback=self._feedback_cb)
        future.add_done_callback(lambda f: self._response_cb(f, my_seq))

        while rclpy.ok() and not self._goal_done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._now() - self._goal_start_time > GOAL_TIMEOUT:
                self.get_logger().warn('Goal timeout')
                return False

        # 真到达校验：Nav2 报 SUCCEEDED 但人还在远处 = 假成功，按失败处理并拉黑该点
        if self._last_status in (4, 6) and not self._reached(x, y, ARRIVE_TOL):
            p = self._get_pose()
            d = math.hypot(p[0] - x, p[1] - y) if p else -1
            self.get_logger().warn(
                f'假成功: Nav2 报 status={self._last_status}，但距目标还有 {d:.1f} m，按失败处理')
            self._last_status = -2
        self.get_logger().info(f'Goal result status={self._last_status}')
        # status=4 SUCCEEDED, status=6 ABORTED(preempted by next goal) - both OK
        return self._last_status in (4, 6)

    def _reached(self, gx, gy, tol):
        p = self._get_pose()
        if p is None:
            return False
        return math.hypot(p[0] - gx, p[1] - gy) <= tol

    def _response_cb(self, future, seq):
        if seq != self._goal_seq:
            return
        h = future.result()
        if not h.accepted:
            self.get_logger().warn('Goal rejected')
            self._goal_done = True
            self._last_status = -1
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = h.get_result_async()
        self._get_result_future.add_done_callback(lambda f: self._result_cb(f, seq))

    def _feedback_cb(self, msg):
        d = msg.feedback.distance_remaining
        if d is not None:
            elapsed = self._now() - self._goal_start_time
            # 不能只信 distance_remaining：目标在地图外/不可达时它是假值，
            # 会让我们把没走到的目标判成成功，然后反复发同一个点。
            if elapsed > 2.0 and self._reached(self._goal_xy[0], self._goal_xy[1], 0.8):
                self._goal_done = True
                self._last_status = 4
                return
            if self._prev_dist is not None and abs(d - self._prev_dist) < 0.1:
                if self._stuck_start is None:
                    self._stuck_start = self._now()
                elif self._now() - self._stuck_start > STUCK_TIMEOUT:
                    self.get_logger().warn('Stuck! Recovery...')
                    self._stuck_diag()
                    self._cancel_goal()
                    self._goal_done = True
                    if self._is_fallen():
                        # 翻倒了：倒着发速度指令毫无意义，只会一直空转刷 Stuck。
                        # 大声报警并给出翻回命令，让人（或脚本）把狗翻回来。
                        self._last_status = -4
                        if not self._fall_logged:
                            self._fall_logged = True
                            self._log_fall()
                        return
                    self._last_status = -3
                    self._recovery()
                    return
            else:
                self._stuck_start = None
            self._prev_dist = d

    def _result_cb(self, future, seq):
        if seq != self._goal_seq:
            return
        r = future.result()
        self._last_status = r.status
        self._goal_done = True

    def _cancel_goal(self):
        try:
            self._client.cancel_all_goals()
        except Exception:
            pass

    def _recovery(self):
        self.get_logger().info('Smart recovery...')
        pose = self._get_pose()
        if pose is None:
            return

        # 动作降温：原来是倒车 -0.3m/s 同时 1.5rad/s 转向 + 原地 2.0rad/s 猛转，
        # 在树根/坡上很容易把四足掀翻（实测 go2_3 翻了 110 次，多半就是这么来的）
        for turn in [1.0, -1.0]:
            self._drive(-0.15, turn * 0.8, RECOVERY_DURATION)
            new_pose = self._get_pose()
            if new_pose and math.hypot(new_pose[0] - pose[0], new_pose[1] - pose[1]) > 0.15:
                self.get_logger().info('Recovery succeeded')
                return

        # Try turning then forward
        for turn in [1.0, -1.0]:
            self._drive(0.0, turn * 1.0, RECOVERY_DURATION)
            self._drive(0.3, 0.0, RECOVERY_DURATION)

        self.get_logger().warn('Recovery completed')

    def _split_box(self, cell):
        """把大 box 切成约 cell 米见方的小块（列表按行优先）。"""
        w = self.BOUND_X_MAX - self.BOUND_X_MIN
        h = self.BOUND_Y_MAX - self.BOUND_Y_MIN
        nx = max(1, int(round(w / cell)))
        ny = max(1, int(round(h / cell)))
        cells = []
        for i in range(nx):
            for j in range(ny):
                x0 = self.BOUND_X_MIN + i * w / nx
                x1 = self.BOUND_X_MIN + (i + 1) * w / nx
                y0 = self.BOUND_Y_MIN + j * h / ny
                y1 = self.BOUND_Y_MIN + (j + 1) * h / ny
                cells.append((x0, x1, y0, y1))
        return cells

    def _split_by_connectivity(self):
        """连通域分析把 Box 切成子区域，返回 [{id, bounds, area, centroid, spawn_point}]。

        实现要点：
          · 用 numpy + scipy.ndimage.label（C 实现）替代纯 Python BFS ——
            Box 1 是 150x72m @5cm = 432 万格，纯 Python 遍历要几十秒，
            而每个子区域完成后都要重算一次，会拖垮整个探索。
          · 先降采样到约 1m 一格（几万格），毫秒级完成；切出来的边界是
            近似值，对"先探哪块"的调度完全够用。
        """
        import numpy as np
        from scipy import ndimage

        m = self._map
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y

        # Box 在栅格上的范围：不夹取到地图！
        # 分区要覆盖【整个 Box】，地图还没覆盖到的部分按"未知"处理；
        # 否则切出来的子区域总面积会明显小于 Big Box（RViz 里对不上红框）。
        gx0, gy0 = self._raw_idx(self.BOUND_X_MIN, self.BOUND_Y_MIN)
        gx1, gy1 = self._raw_idx(self.BOUND_X_MAX, self.BOUND_Y_MAX)
        gx0, gx1 = min(gx0, gx1), max(gx0, gx1)
        gy0, gy1 = min(gy0, gy1), max(gy0, gy1)
        gw, gh = gx1 - gx0 + 1, gy1 - gy0 + 1
        if gw <= 2 or gh <= 2:
            return []

        arr = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        box = np.full((gh, gw), -1, dtype=np.int16)      # 默认未知
        ix0, iy0 = max(0, gx0), max(0, gy0)
        ix1, iy1 = min(m.info.width - 1, gx1), min(m.info.height - 1, gy1)
        if ix1 >= ix0 and iy1 >= iy0:
            box[iy0 - gy0:iy1 - gy0 + 1, ix0 - gx0:ix1 - gx0 + 1] = \
                arr[iy0:iy1 + 1, ix0:ix1 + 1]

        # 降采样到 ~1m 一格；一个降采样格只要含任一 free/unknown 就算可通行，
        # 避免把细窄走廊误判成断开
        F = max(1, int(round(1.0 / res)))
        gh2, gw2 = gh // F, gw // F
        if gh2 < 3 or gw2 < 3:
            F, gh2, gw2 = 1, gh, gw
        b = box[:gh2 * F, :gw2 * F]
        p_ds = ((b == 0) | (b == -1)).reshape(gh2, F, gw2, F).any(axis=(1, 3))

        cell_m = res * F
        max_area = max(1, int((SUBREGION_TARGET_M / cell_m) ** 2))
        min_area = max(1, int((SUBREGION_MIN_M / cell_m) ** 2))

        labels, n = ndimage.label(p_ds, structure=np.ones((3, 3), dtype=np.uint8))
        if n == 0:
            return []
        counts = np.bincount(labels.ravel())
        slices = ndimage.find_objects(labels)

        subregions = []
        rid = 0
        for idx in range(1, n + 1):
            area = int(counts[idx])
            if area < min_area:
                continue
            sl = slices[idx - 1]
            if sl is None:
                continue
            ys, xs = sl
            sub_mask = labels[sl] == idx
            ys_i, xs_i = np.nonzero(sub_mask)
            pixels = list(zip(xs_i.tolist(), ys_i.tolist()))

            if area > max_area:
                # 二次切割：界面对齐到该连通域 bbox 的原点
                ox_ds = ox + (gx0 + xs.start * F) * res
                oy_ds = oy + (gy0 + ys.start * F) * res
                sub = self._split_large_region(
                    pixels, 0, 0, cell_m, ox_ds, oy_ds, rid, area, max_area, 0)
                subregions.extend(sub)
                rid += len(sub)
                continue

            # 用"比例映射"而不是 i*F*res：降采样会把 Box 截到 F 的整数倍，
            # 直接乘会在边缘留下最多 0.95m 的空隙（RViz 里红框和子区域对不齐）
            bx0, by0 = ox + gx0 * res, oy + gy0 * res
            bw, bh = gw * res, gh * res
            wx0 = bx0 + (xs.start / gw2) * bw
            wx1 = bx0 + (xs.stop / gw2) * bw
            wy0 = by0 + (ys.start / gh2) * bh
            wy1 = by0 + (ys.stop / gh2) * bh
            subregions.append({
                'id': rid,
                'bounds': (wx0, wx1, wy0, wy1),
                'area': area,
                'centroid': ((wx0 + wx1) / 2.0, (wy0 + wy1) / 2.0),
                'spawn_point': None,
            })
            rid += 1

        # 统一修正 spawn_point：按 bounds 在图上找自由格；找不到 = 进不去（会被调度跳过）
        for sr in subregions:
            x0, x1, y0, y1 = sr['bounds']
            rc0, rr0 = self._raw_idx(x0, y0)
            rc1, rr1 = self._raw_idx(x1, y1)
            rc0, rc1 = min(rc0, rc1), max(rc0, rc1)
            rr0, rr1 = min(rr0, rr1), max(rr0, rr1)
            # 完全在地图之外：现在没法进去（地图长大后重算会再评估）
            if (rc1 < 0 or rr1 < 0 or rc0 > m.info.width - 1
                    or rr0 > m.info.height - 1):
                sr['spawn_point'] = None
                continue
            c0, c1 = max(0, rc0), min(m.info.width - 1, rc1)
            r0, r1 = max(0, rr0), min(m.info.height - 1, rr1)
            sub = arr[r0:r1 + 1, c0:c1 + 1]
            fy, fx = np.nonzero(sub == 0)
            if len(fx) == 0:
                sr['spawn_point'] = None
                continue
            ccx, ccy = (c0 + c1) / 2.0, (r0 + r1) / 2.0
            k = int(np.argmin((fx + c0 - ccx) ** 2 + (fy + r0 - ccy) ** 2))
            sr['spawn_point'] = ((c0 + int(fx[k])) * res + ox,
                                 (r0 + int(fy[k])) * res + oy)

        self.get_logger().info(
            f'Box {self._box_id} 连通域分析: {len(subregions)} 个子区域 '
            f'(降采样 x{F} -> {gw2}x{gh2} 格, 目标 {SUBREGION_TARGET_M:.0f}m)')
        return subregions

    def _find_subregion_spawn(self, pixels, gx0, gy0, res, ox, oy):
        """在子区域像素列表中找离质心最近的 free 格子作为入口点。"""
        m = self._map
        cx = sum(p[0] for p in pixels) / len(pixels) + gx0
        cy = sum(p[1] for p in pixels) / len(pixels) + gy0
        best_dist = float('inf')
        best_xy = None
        for px, py in pixels:
            wx = (px + gx0) * res + ox
            wy = (py + gy0) * res + oy
            cost = m.data[(py + gy0) * m.info.width + (px + gx0)]
            if cost == 0:  # confirmed free
                d = math.hypot(px + gx0 - cx, py + gy0 - cy)
                if d < best_dist:
                    best_dist = d
                    best_xy = (wx, wy)
        if best_xy:
            return best_xy
        # fallback：返回质心
        return (cx * res + ox, cy * res + oy)

    def _split_large_region(self, pixels, gx0, gy0, res, ox, oy, base_id, total_area,
                            max_area=None, depth=0):
        """对过大子区域做二次切割：沿最长轴在最窄处切分。"""
        # min_area 是本方法要用的局部阈值（_split_by_connectivity 里的同名变量在别处作用域）
        min_area = max(1, int((SUBREGION_MIN_M / res) ** 2))
        # 找像素的边界框
        pxs = [p[0] for p in pixels]
        pys = [p[1] for p in pixels]
        min_px, max_px = min(pxs), max(pxs)
        min_py, max_py = min(pys), max(pys)
        pw, ph = max_px - min_px + 1, max_py - min_py + 1

        # 沿最长轴切
        if pw >= ph:
            # 沿 x 轴切：找 x 方向最窄的列
            col_counts = {}
            for px, py in pixels:
                col_counts.setdefault(px, 0)
                col_counts[px] += 1
            # 滑动窗口找最小密度位置
            mid = (min_px + max_px) // 2
            best_x = mid
            best_key = (float('inf'), 0)
            # 只在中间一半找切点：均匀区域里各列密度相同，若允许全范围搜索，
            # 严格小于的比较会一直选中最左边，切出一条条细边
            lo = min_px + max(2, pw // 4)
            hi = max_px - max(2, pw // 4)
            for x in range(lo, hi + 1):
                density = col_counts.get(x, 0) + col_counts.get(x - 1, 0) + col_counts.get(x + 1, 0)
                key = (density, abs(x - mid))     # 同等密度取更靠近中点的
                if key < best_key:
                    best_key = key
                    best_x = x
            left = [p for p in pixels if p[0] <= best_x]
            right = [p for p in pixels if p[0] > best_x]
        else:
            row_counts = {}
            for px, py in pixels:
                row_counts.setdefault(py, 0)
                row_counts[py] += 1
            mid = (min_py + max_py) // 2
            best_y = mid
            best_key = (float('inf'), 0)
            lo = min_py + max(2, ph // 4)
            hi = max_py - max(2, ph // 4)
            for y in range(lo, hi + 1):
                density = row_counts.get(y, 0) + row_counts.get(y - 1, 0) + row_counts.get(y + 1, 0)
                key = (density, abs(y - mid))
                if key < best_key:
                    best_key = key
                    best_y = y
            left = [p for p in pixels if p[1] <= best_y]
            right = [p for p in pixels if p[1] > best_y]

        results = []
        for group in [left, right]:
            if len(group) < min_area:
                continue
            # 还太大就继续切（原来只切一刀就返回，150x72m 的 Box 永远只能得到 2 块）
            if (max_area is not None and len(group) > max_area and depth < 10
                    and len(group) < len(pixels)):
                results.extend(self._split_large_region(
                    group, gx0, gy0, res, ox, oy, base_id + len(results),
                    len(group), max_area, depth + 1))
                continue
            ppxs = [p[0] for p in group]
            ppys = [p[1] for p in group]
            wx0 = min(ppxs) * res + ox
            wx1 = (max(ppxs) + 1) * res + ox
            wy0 = min(ppys) * res + oy
            wy1 = (max(ppys) + 1) * res + oy
            cx_w = (min(ppxs) + max(ppxs)) / 2.0 * res + ox
            cy_w = (min(ppys) + max(ppys)) / 2.0 * res + oy
            spawn = self._find_subregion_spawn(group, gx0, gy0, res, ox, oy)
            results.append({
                'id': base_id + len(results),
                'bounds': (wx0, wx1, wy0, wy1),
                'area': len(group),
                'centroid': (cx_w, cy_w),
                'spawn_point': spawn,
            })
        return results

    def _cluster_frontiers(self, frontiers):
        """网格聚类：把相邻的前沿点合并成聚类质心。"""
        if not frontiers:
            return []
        # 网格量化
        grid = {}
        for f in frontiers:
            key = (int(f[0] / CLUSTER_GRID_SIZE), int(f[1] / CLUSTER_GRID_SIZE))
            grid.setdefault(key, []).append(f)

        centroids = []
        for key, pts in grid.items():
            # 加权平均（距离远的权重大，避免被近处点拉偏）
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            centroids.append((cx, cy, len(pts)))  # 附带聚类大小
        return centroids

    def _evaluate_utility(self, robot_pos, candidate, heading=None):
        """计算候选点的信息增益效用。
        用扇形视野模拟：从候选点展开 180° 扇形，数能新增多少 unknown 格子。
        返回 (utility, info_gain)
        """
        m = self._map
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y

        if heading is None:
            # 朝向 = 从机器人指向候选点的方向
            dx = candidate[0] - robot_pos[0]
            dy = candidate[1] - robot_pos[1]
            heading = math.atan2(dy, dx)

        # 路径代价：简单用直线距离（后续可换 A*）
        path_cost = math.hypot(candidate[0] - robot_pos[0], candidate[1] - robot_pos[1])

        # 扇形视野模拟
        info_gain = 0
        sensor_rCells = int(SENSOR_RADIUS / res)
        angle_start = heading - SENSOR_ANGLE / 2
        angle_step = SENSOR_ANGLE / SENSOR_RAYS

        cx_idx, cy_idx = self._coord_to_idx(candidate[0], candidate[1])

        for i in range(SENSOR_RAYS):
            angle = angle_start + i * angle_step
            dx = math.cos(angle)
            dy = math.sin(angle)
            # 沿射线逐格扫描
            for r in range(1, sensor_rCells + 1):
                tx = int(cx_idx + dx * r)
                ty = int(cy_idx + dy * r)
                if tx < 0 or tx >= m.info.width or ty < 0 or ty >= m.info.height:
                    break
                cell_val = m.data[ty * m.info.width + tx]
                if cell_val >= 100:  # occupied，射线终止
                    break
                if cell_val == -1:  # unknown，计入信息增益
                    info_gain += 1

        # 效用 = 信息增益 / (路径代价 + 1)
        utility = info_gain / (path_cost + 1.0)
        return utility, info_gain

    def _set_bounds(self, c):
        self.BOUND_X_MIN, self.BOUND_X_MAX, self.BOUND_Y_MIN, self.BOUND_Y_MAX = c

    def _publish_region(self):
        m = Marker()
        m.header.frame_id = f'{self._robot_name}/map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'region'
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = self.BOUND_X_MAX - self.BOUND_X_MIN
        m.scale.y = self.BOUND_Y_MAX - self.BOUND_Y_MIN
        m.scale.z = 0.05
        m.color.a = 0.2
        m.color.g = 1.0
        m.pose.orientation.w = 1.0
        m.pose.position.x = (self.BOUND_X_MIN + self.BOUND_X_MAX) / 2
        m.pose.position.y = (self.BOUND_Y_MIN + self.BOUND_Y_MAX) / 2
        m.pose.position.z = 0.0
        from geometry_msgs.msg import Point
        m.points.append(Point(x=0.0, y=0.0, z=0.0))
        self._region_pub.publish(m)

    def _publish_bigbox(self):
        """发布整个大 box 的红色边框（LINE_STRIP），只在探索开始时调一次。"""
        if self._full_bounds is None:
            return
        x0, x1, y0, y1 = self._full_bounds
        m = Marker()
        m.header.frame_id = f'{self._robot_name}/map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bigbox'
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.15
        m.color.r = 1.0
        m.color.a = 0.8
        m.pose.orientation.w = 1.0
        from geometry_msgs.msg import Point
        # 顺时针画矩形
        for px, py in [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]:
            m.points.append(Point(x=px, y=py, z=0.05))
        self._bigbox_pub.publish(m)

    def _republish_markers(self):
        """定时重发子区域 marker：一是让 RViz 任何时候进来都能看到，
        二是子区域状态（当前/待探索/已完成）变了能及时反映。"""
        if self._last_sub is not None:
            self._publish_subregions(self._last_sub, self._last_sub_cur, self._last_sub_done,
                                 self._last_sub_bounds)

    def _publish_subregions(self, subregions, current_id, done_ids, done_bounds=None):
        """发布所有子区域边框，颜色区分状态：
        - 当前子区域: 亮绿色 (0.0, 1.0, 0.0)
        - 待探索:     暗蓝色 (0.3, 0.3, 0.8)
        - 已完成:     暗灰色 (0.5, 0.5, 0.5)
        """
        m = Marker()
        m.header.frame_id = f'{self._robot_name}/map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'subregions'
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.12
        m.pose.orientation.w = 1.0

        # 完成判据：bounds 精确匹配（同一轮划分内）或"基本探完"(unknown 很少)。
        # 只按 bounds 匹配是不行的 —— 每完成一块就会用最新地图重算划分，
        # 新 bounds 和旧的完全对不上，已探完的区域会一直显示成"待探索"。
        done_eff = set(done_ids)
        db = done_bounds or ()
        for sr in subregions:
            if sr['id'] in done_eff:
                continue
            # (a) 质心落在"已完成"的矩形里（重算划分后新块仍能识别为已完成）
            cx, cy = sr['centroid']
            if any(b[0] <= cx <= b[1] and b[2] <= cy <= b[3] for b in db):
                done_eff.add(sr['id'])
                continue
            # (b) 按最新地图这块基本探完了
            if '_unknown_density' not in sr:
                sr['_unknown_density'] = self._estimate_unknown_density(sr)
            if sr['_unknown_density'] < 0.03:
                done_eff.add(sr['id'])

        for sr in subregions:
            sid = sr['id']
            x0, x1, y0, y1 = sr['bounds']
            z = 0.05

            if sid == current_id:
                r, g, b, a = 0.0, 1.0, 0.0, 0.9  # 亮绿
            elif sid in done_eff:
                r, g, b, a = 0.5, 0.5, 0.5, 0.5  # 暗灰
            else:
                r, g, b, a = 0.3, 0.3, 0.8, 0.6  # 暗蓝

            corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            for i in range(4):
                p1 = Point(x=corners[i][0], y=corners[i][1], z=z)
                p2 = Point(x=corners[(i + 1) % 4][0], y=corners[(i + 1) % 4][1], z=z)
                m.points.append(p1)
                m.points.append(p2)
                m.colors.append(ColorRGBA(r=r, g=g, b=b, a=a))
                m.colors.append(ColorRGBA(r=r, g=g, b=b, a=a))

            # 子区域编号标签（用 text marker 不太方便，这里用颜色区分就够了）

        # 缓存下来给定时器重发
        self._last_sub = subregions
        self._last_sub_cur = current_id
        self._last_sub_done = set(done_ids)
        self._last_sub_bounds = set() if not done_bounds else set(done_bounds)
        self._subregions_pub.publish(m)

    def _publish_points(self, pub, pts, scale, rgb, ns):
        m = Marker()
        m.header.frame_id = f'{self._robot_name}/map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.a = 0.9
        m.color.r, m.color.g, m.color.b = rgb
        m.pose.orientation.w = 1.0
        for p in pts:
            from geometry_msgs.msg import Point
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.15
            m.points.append(pt)
        pub.publish(m)

    def run(self):
        self.get_logger().info(f'Waiting for map...')
        start = time.time()
        while not self._map_ready and time.time() - start < 180:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not self._map_ready:
            self.get_logger().error('No map received')
            return False

        self.get_logger().info(f'Waiting for action server...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Action server unavailable')
            return False

        self.get_logger().info('Starting exploration...')

        full = (self.BOUND_X_MIN, self.BOUND_X_MAX, self.BOUND_Y_MIN, self.BOUND_Y_MAX)
        self._full_bounds = full
        self._publish_bigbox()

        # 连通域子区域划分
        subregions = self._split_by_connectivity()
        if not subregions:
            self.get_logger().warn('连通域分析无子区域，回退到整块探索')
            self._cell_total = 1
            self._cell_order = 1
            cov_pct = self._explore_cell()
            self.get_logger().info('Exploration finished! Saving map...')
            self._save_result(cov_pct, -1, -1)
            return True

        self._cell_total = len(subregions)
        remaining = list(subregions)
        done_bounds = set()          # 用 bounds 而不是 id：重算划分后 id 会变
        no_cand_rounds = 0           # 连续"没有可用子区域"的轮数
        cell_fails = {}              # bounds -> 进不去的次数
        stagnant = 0                 # 连续"没进展"的块数
        last_cov = self._coverage() * 100.0
        visited_pts = []             # 最近访问过的子区域质心（避免回头重复探索）
        self._last_dir = None        # 上一次的行进方向（用于航向一致性）
        order = 0

        # 初始发布所有子区域（全部为待探索蓝色）
        self._publish_subregions(subregions, current_id=-1, done_ids=set(),
                                 done_bounds=done_bounds)

        while rclpy.ok():
            # 没有待探索子区域时，用最新地图重算一次划分（地图会随探索长大）
            if not remaining:
                self._set_bounds(full)
                new_subs = self._split_by_connectivity()
                remaining = [s for s in new_subs
                             if s['bounds'] not in done_bounds]
                if not remaining:
                    break
                subregions = new_subs

            pose = self._get_pose()
            if pose is None:
                rclpy.spin_once(self, timeout_sec=0.5)
                continue

            # 候选过滤（最近邻链式走访 / TSP 近似）：
            #   · spawn_point 为 None = 子区域内没有任何已知自由格 -> 进不去，跳过
            #   · unknown_density < 0.03 = 已探完 -> 跳过（重算后不重复探索）
            cands = []
            for sr in remaining:
                sr['_dist'] = math.hypot(sr['centroid'][0] - pose[0],
                                         sr['centroid'][1] - pose[1])
                sr['_unknown_density'] = self._estimate_unknown_density(sr)
                if sr['_unknown_density'] < 0.03:
                    done_bounds.add(sr['bounds'])      # 真的探完了
                    continue
                # 没有已知自由格入口的块也保留：开局地图还没长过来时，
                # 如果直接排除会导致"一块候选都没有"而秒退（go2_2 就这么退过）。
                # 改成降权，让有入口的优先，实在没有就派它去推未知区域。
                sr['_no_spawn'] = 0.0 if sr.get('spawn_point') else 1.0
                cands.append(sr)

            if not cands:
                # 所有子区域都进不去：等地图长大再重算；连续几轮都没有就收工
                no_cand_rounds += 1
                self.get_logger().info(
                    f'本轮无可用子区域 ({no_cand_rounds}/3)')
                if no_cand_rounds >= 3:
                    self.get_logger().info('连续多轮没有可去的子区域，探索结束')
                    break
                remaining = []
                rclpy.spin_once(self, timeout_sec=2.0)
                continue
            no_cand_rounds = 0

            # 打分（越小越好）：
            #   1.0 × 归一化距离         —— 最近邻，主要行程成本
            #   0.3 × 转向惩罚           —— 与上一条行进方向夹角越大越差，避免来回折返
            #   0.3 × 最近访问惩罚       —— 15m 内刚探过的区域降权
            #  -0.25 × 未知密度          —— 未知多的地方优先
            dmax = max([s['_dist'] for s in cands] + [1e-6])
            for sr in cands:
                d_norm = sr['_dist'] / dmax
                # 航向一致性
                if self._last_dir is not None and sr['_dist'] > 1e-3:
                    vx = (sr['centroid'][0] - pose[0]) / sr['_dist']
                    vy = (sr['centroid'][1] - pose[1]) / sr['_dist']
                    cosang = self._last_dir[0] * vx + self._last_dir[1] * vy
                    turn_pen = (1.0 - cosang) / 2.0
                else:
                    turn_pen = 0.0
                # 最近访问惩罚
                recent_pen = 0.0
                for rx, ry in visited_pts:
                    if math.hypot(sr['centroid'][0] - rx,
                                  sr['centroid'][1] - ry) < 15.0:
                        recent_pen = 1.0
                        break
                sr['_turn_pen'] = turn_pen
                sr['_recent_pen'] = recent_pen
                sr['_score'] = (d_norm + 0.3 * turn_pen
                                + 0.3 * recent_pen
                                + 0.5 * sr.get('_no_spawn', 0.0)
                                - 0.25 * sr['_unknown_density'])
            cands.sort(key=lambda s: s['_score'])

            cur = cands.pop(0)

            # 记录行进方向，供下一轮航向一致性使用
            if cur['_dist'] > 1e-3:
                self._last_dir = ((cur['centroid'][0] - pose[0]) / cur['_dist'],
                                  (cur['centroid'][1] - pose[1]) / cur['_dist'])

            order += 1
            self._cell_order = order
            self._set_bounds(cur['bounds'])

            # 发布子区域可视化：当前=绿色，待探索=蓝色，已完成=灰色
            done_ids = {s['id'] for s in subregions if s['bounds'] in done_bounds}
            self._publish_subregions(subregions, current_id=cur['id'], done_ids=done_ids,
                                     done_bounds=done_bounds)

            self.get_logger().info(
                f'=== 子区域 {order} (id={cur["id"]}, 剩余 {len(cands)}): '
                f'x[{cur["bounds"][0]:.1f},{cur["bounds"][1]:.1f}] '
                f'y[{cur["bounds"][2]:.1f},{cur["bounds"][3]:.1f}] '
                f'area={cur["area"]} dist={cur["_dist"]:.1f} '
                f'density={cur["_unknown_density"]:.2f} '
                f'turn={cur["_turn_pen"]:.2f} recent={cur["_recent_pen"]:.0f} '
                f'no_spawn={cur.get("_no_spawn", 0):.0f} score={cur["_score"]:.3f}')

            cov_before = self._coverage() * 100.0
            self._explore_cell()
            cov_after = self._coverage() * 100.0
            cell_dens = self._estimate_unknown_density({'bounds': cur['bounds']})

            if cov_after - cov_before < 0.5 and cell_dens > 0.5:
                # 这块基本没动（多半是没进去 / 破门也失败）：先不标记完成，稍后重试
                n = cell_fails.get(cur['bounds'], 0) + 1
                cell_fails[cur['bounds']] = n
                if n >= 2:
                    done_bounds.add(cur['bounds'])
                    self.get_logger().info(
                        f'子区域 {order} 试了 {n} 次都进不去（未知度 {cell_dens:.2f}），放弃')
                else:
                    self.get_logger().info(
                        f'子区域 {order} 没进展（未知度 {cell_dens:.2f}），稍后重试')
            else:
                done_bounds.add(cur['bounds'])

            # 全局停滞保护：连续多个块都没让整体覆盖率上升，说明进不去了，收工
            if cov_after - last_cov < 0.5:
                stagnant += 1
            else:
                stagnant = 0
            last_cov = cov_after
            if stagnant >= 6:
                self.get_logger().info(
                    f'连续 {stagnant} 个块没有进展，探索结束. 整体覆盖率 {cov_after:.1f}%')
                break
            visited_pts.append(cur['centroid'])
            if len(visited_pts) > 8:
                visited_pts.pop(0)

            # 切回全 Box 算整体覆盖率
            self._set_bounds(full)
            overall = self._coverage() * 100.0
            self.get_logger().info(
                f'--- 子区域 {order} 完成, 整体覆盖率: {overall:.1f}% ---')

            # 全局无前沿检查
            if overall > 0 and self._no_global_frontier():
                self.get_logger().info('全局无前沿，探索结束')
                break

            # 地图变了，重新切分：已完成的按 bounds 剔除，剩下的重新排队
            new_subs = self._split_by_connectivity()
            if new_subs:
                subregions = new_subs
                self._cell_total = len(subregions)
                remaining = [s for s in subregions if s['bounds'] not in done_bounds]
                done_ids = {s['id'] for s in subregions if s['bounds'] in done_bounds}
                self._publish_subregions(subregions, current_id=-1, done_ids=done_ids,
                                         done_bounds=done_bounds)

        self._set_bounds(full)
        cov_pct = self._coverage() * 100.0
        self.get_logger().info('Exploration finished! Saving map...')
        self._save_result(cov_pct, -1, -1)
        return True

    def _estimate_unknown_density(self, subregion):
        """子区域内 unknown 的比例。

        注意：地图还没覆盖到的部分按"未知"计入分母。原来用 _coord_to_idx（会夹取到
        地图边缘），完全在地图外的子区域会退化成一两个边缘格 -> 密度算成 0 ->
        被判定"已探完"而跳过，go2_2 就是这样在 21.8% 提前收工的。
        """
        m = self._map
        if m is None:
            return 0.5
        x0, x1, y0, y1 = subregion['bounds']
        rc0, rr0 = self._raw_idx(x0, y0)
        rc1, rr1 = self._raw_idx(x1, y1)
        rc0, rc1 = min(rc0, rc1), max(rc0, rc1)
        rr0, rr1 = min(rr0, rr1), max(rr0, rr1)
        total_box = max(1, (rc1 - rc0 + 1) * (rr1 - rr0 + 1))
        gx0, gx1 = max(0, rc0), min(m.info.width - 1, rc1)
        gy0, gy1 = max(0, rr0), min(m.info.height - 1, rr1)
        if gx1 < gx0 or gy1 < gy0:
            return 1.0          # 完全在地图外 = 全未知
        gw, gh = gx1 - gx0 + 1, gy1 - gy0 + 1
        import numpy as np
        arr = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        sub = arr[gy0:gy1 + 1, gx0:gx1 + 1]
        total = sub.size
        if total == 0:
            return 0.5
        unknown = int(np.count_nonzero(sub == -1))
        # 地图外的格子也算未知
        outside = total_box - int(sub.size)
        return (unknown + outside) / float(total_box)

    def _no_global_frontier(self):
        """检查整个 Box 是否还有前沿。"""
        m = self._map
        if m is None:
            return False
        pose = self._get_pose()
        if pose is None:
            return False
        frontiers = self._detect_frontiers(pose, MAX_GOAL_DISTANCE * 4)
        # 只统计 Box 内的
        for f in frontiers:
            if (self.BOUND_X_MIN <= f[0] <= self.BOUND_X_MAX and
                    self.BOUND_Y_MIN <= f[1] <= self.BOUND_Y_MAX):
                return False
        return True

    def _overall_cov(self):
        """计算整个大 box 的覆盖率（临时切换到 full_bounds 再切回来）。"""
        if self._full_bounds is None:
            return self._coverage() * 100.0
        saved = (self.BOUND_X_MIN, self.BOUND_X_MAX, self.BOUND_Y_MIN, self.BOUND_Y_MAX)
        self._set_bounds(self._full_bounds)
        cov = self._coverage() * 100.0
        self._set_bounds(saved)
        return cov

    def _overall_cov_mapped(self):
        """整个 box 的旧口径覆盖率（已知 / 已建图∩box），只作对照。"""
        if self._full_bounds is None:
            return self._coverage_mapped() * 100.0
        saved = (self.BOUND_X_MIN, self.BOUND_X_MAX, self.BOUND_Y_MIN, self.BOUND_Y_MAX)
        self._set_bounds(self._full_bounds)
        cov = self._coverage_mapped() * 100.0
        self._set_bounds(saved)
        return cov

    def _explore_cell(self):
        """探索当前 self.BOUND_* 范围内的区域，使用信息增益选点。"""
        self._publish_region()
        blacklisted = set()
        goal_count = 0
        fail_count = 0
        no_frontier_rounds = 0
        cov_pct = self._coverage() * 100.0

        while rclpy.ok():
            pose = self._get_pose()
            if pose is None:
                rclpy.spin_once(self, timeout_sec=0.5)
                continue

            # 翻倒状态下继续发目标没有意义，等被翻回来再继续
            if self._is_fallen():
                if not self._fall_logged:
                    self._fall_logged = True
                    self._log_fall()
                rclpy.spin_once(self, timeout_sec=2.0)
                continue
            self._fall_logged = False

            cov = self._coverage()
            cov_pct = cov * 100
            self._publish_region()

            # 检查子区域覆盖率
            if cov >= SUBREGION_COVERAGE_TARGET:
                self.get_logger().info(
                    f'[选块#{self._cell_order} 划分{self._cell_total}块] '
                    f'子区域覆盖率达标 ({cov_pct:.1f}% >= {SUBREGION_COVERAGE_TARGET*100:.0f}%)')
                break

            # 检查失败次数
            if fail_count >= 5:
                self.get_logger().info(
                    f'[选块#{self._cell_order} 划分{self._cell_total}块] '
                    f'连续失败过多，切换子区域. Coverage: {cov_pct:.1f}%')
                break

            # 检测前沿
            frontiers = []
            radius = self.MAX_GOAL_DISTANCE
            while radius <= 128.0:
                frontiers = self._detect_frontiers(pose, radius)
                if frontiers:
                    break
                radius *= 2.0

            # 过滤：只保留子区域内、未被拉黑的前沿
            available_raw = []
            near = []
            for f in frontiers:
                if not (self.BOUND_X_MIN <= f[0] <= self.BOUND_X_MAX and
                        self.BOUND_Y_MIN <= f[1] <= self.BOUND_Y_MAX):
                    continue
                key = (round(f[0], 1), round(f[1], 1))
                if key in blacklisted:
                    continue
                dist = math.hypot(f[0] - pose[0], f[1] - pose[1])
                if dist < MIN_GOAL_DISTANCE:
                    near.append(f)
                else:
                    available_raw.append(f)

            # 前沿聚类
            clusters = self._cluster_frontiers(available_raw)

            # 计算每个聚类质心的效用
            candidates = []
            for cx, cy, size in clusters:
                key = (round(cx, 1), round(cy, 1))
                if key in blacklisted:
                    continue
                dist = math.hypot(cx - pose[0], cy - pose[1])
                if dist < MIN_GOAL_DISTANCE:
                    near.append((cx, cy))
                    continue
                utility, info_gain = self._evaluate_utility(pose, (cx, cy))
                candidates.append((cx, cy, utility, info_gain, size))

            # 处理只有近处前沿的情况
            if not candidates and near:
                f = max(near, key=lambda f: math.hypot(f[0] - pose[0], f[1] - pose[1]))
                dx, dy = f[0] - pose[0], f[1] - pose[1]
                d = math.hypot(dx, dy) or 1e-6
                ext = MIN_GOAL_DISTANCE + 1.0
                gx = min(max(pose[0] + dx / d * ext, self.BOUND_X_MIN), self.BOUND_X_MAX)
                gy = min(max(pose[1] + dy / d * ext, self.BOUND_Y_MIN), self.BOUND_Y_MAX)
                gx, gy = self._find_nearest_free(gx, gy)
                utility, info_gain = self._evaluate_utility(pose, (gx, gy))
                candidates = [(gx, gy, utility, info_gain, 1)]

            # 无前沿处理
            used_fallback = False
            if not candidates:
                used_fallback = True
                no_frontier_rounds += 1
                self.get_logger().info(
                    f'[选块#{self._cell_order} 划分{self._cell_total}块] '
                    f'无前沿可去 ({no_frontier_rounds}/{NO_FRONTIER_ROUNDS}) '
                    f'子区域: {cov_pct:.1f}% | 整体: {self._overall_cov():.1f}% | 整体(旧口径): {self._overall_cov_mapped():.1f}%')
                if no_frontier_rounds >= NO_FRONTIER_ROUNDS:
                    self.get_logger().info(
                        f'连续 {NO_FRONTIER_ROUNDS} 轮无前沿，子区域结束. '
                        f'Coverage: {cov_pct:.1f}%')
                    break
                # 破门：如果当前子区域基本全是未知，前沿检测永远产不出点
                # （前沿 = 自由格与未知格的交界，而全未知区里没有自由格），
                # 必须主动导航到子区域中心闯进去，进去后才会重新长出前沿。
                # 没有这一步，狗只会在"已经看得见的区域"里打转，永远进不去未知块。
                cell_density = self._estimate_unknown_density(
                    {'bounds': (self.BOUND_X_MIN, self.BOUND_X_MAX,
                                self.BOUND_Y_MIN, self.BOUND_Y_MAX)})
                # 破门目标用"离狗最近的块内入口点"，不要用块中心：
                # 中心可能远在 60+ 米外，穿越未知区域跑那么远基本都会导航失败
                # （实测 go2_1 破门目标 66m -> 反复"进不去"，最终 28.5% 收工）。
                ex = min(max(pose[0], self.BOUND_X_MIN), self.BOUND_X_MAX)
                ey = min(max(pose[1], self.BOUND_Y_MIN), self.BOUND_Y_MAX)
                # 狗已经在块内时，往块中心方向推进一段
                if abs(ex - pose[0]) < 1e-6 and abs(ey - pose[1]) < 1e-6:
                    ccx = (self.BOUND_X_MIN + self.BOUND_X_MAX) / 2.0
                    ccy = (self.BOUND_Y_MIN + self.BOUND_Y_MAX) / 2.0
                    dx, dy = ccx - pose[0], ccy - pose[1]
                    d = math.hypot(dx, dy) or 1e-6
                    step = min(d, MIN_GOAL_DISTANCE + 3.0)
                    ex, ey = pose[0] + dx / d * step, pose[1] + dy / d * step
                ex, ey = self._find_nearest_free(ex, ey)
                d_entry = math.hypot(ex - pose[0], ey - pose[1])
                if cell_density > 0.4 and d_entry > max(MIN_GOAL_DISTANCE, ARRIVE_TOL * 2):
                    self.get_logger().info(
                        f'破门：子区域未知度 {cell_density:.2f}，从最近入口进入 '
                        f'({ex:.1f},{ey:.1f}) 距离 {d_entry:.1f}m')
                    candidates = [(ex, ey, 0, 0, 1)]

            if not candidates:
                # 兜底：去子区域内最近的自由格
                ix = min(max(pose[0], self.BOUND_X_MIN), self.BOUND_X_MAX)
                iy = min(max(pose[1], self.BOUND_Y_MIN), self.BOUND_Y_MAX)
                tx, ty = self._find_nearest_free(ix, iy)
                if math.hypot(tx - pose[0], ty - pose[1]) < max(MIN_GOAL_DISTANCE, ARRIVE_TOL * 2):
                    self.get_logger().info('兜底点就在脚下，本轮不作为')
                    rclpy.spin_once(self, timeout_sec=1.0)
                    continue
                candidates = [(tx, ty, 0, 0, 1)]

            if not used_fallback:
                no_frontier_rounds = 0

            # 选效用最高的候选点
            candidates.sort(key=lambda c: c[2], reverse=True)
            best = candidates[0]
            best_xy = (best[0], best[1])

            # 可视化
            viz_pts = [(c[0], c[1]) for c in candidates]
            self._publish_points(self._frontier_pub, viz_pts, 0.25, (0.0, 1.0, 1.0), 'frontiers')
            self._publish_points(self._goal_pub, [best_xy], 0.5, (1.0, 0.2, 0.0), 'goal')

            self.get_logger().info(
                f'[选块#{self._cell_order} 划分{self._cell_total}块] '
                f'Goal: ({best[0]:.1f}, {best[1]:.1f}) util={best[2]:.1f} gain={best[3]} '
                f'cluster_size={best[4]} | '
                f'子区域: {cov_pct:.1f}% | 整体: {self._overall_cov():.1f}% | 整体(旧口径): {self._overall_cov_mapped():.1f}%')

            goal_count += 1
            if not self._send_goal(best[0], best[1]):
                fail_count += 1
                key = (round(best[0], 1), round(best[1], 1))
                blacklisted.add(key)
                self.get_logger().warn(f'Goal failed ({fail_count}/{goal_count}), blacklisting')
            else:
                fail_count = max(0, fail_count - 1)
                # 破门成功（真的移动/到达）也算有进展，清掉无前沿计数
                no_frontier_rounds = 0

        self.get_logger().info(
            f'[选块#{self._cell_order} 划分{self._cell_total}块] 子区域结束 '
            f'子区域: {cov_pct:.1f}% | 整体: {self._overall_cov():.1f}% | 整体(旧口径): {self._overall_cov_mapped():.1f}%')
        return cov_pct

    def _save_result(self, cov_pct, goal_count, fail_count):
        """把当前地图存成 nav2 的 .pgm + .yaml，另外存一份 json 摘要。"""
        if self._map is None:
            self.get_logger().warn('没有地图可保存')
            return
        prefix = self._save_prefix
        d = os.path.dirname(prefix)
        if d:
            os.makedirs(d, exist_ok=True)

        m = self._map
        w, h = m.info.width, m.info.height
        # OccupancyGrid 原点在左下，PGM 从左上写，所以行要倒过来
        buf = bytearray()
        for row in range(h - 1, -1, -1):
            for v in m.data[row * w:(row + 1) * w]:
                buf.append(254 if v == 0 else (0 if v >= 100 else 205))
        pgm = prefix + '.pgm'
        with open(pgm, 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (w, h))
            f.write(bytes(buf))

        q = m.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaml_path = prefix + '.yaml'
        with open(yaml_path, 'w') as f:
            f.write("image: %s\n" % os.path.basename(pgm))
            f.write("mode: trinary\n")
            f.write("resolution: %s\n" % m.info.resolution)
            f.write("origin: [%s, %s, %s]\n" % (m.info.origin.position.x,
                                                 m.info.origin.position.y, yaw))
            f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

        summary = {
            'robot': self._robot_name,
            'box_id': self._box_id,
            'coverage_pct': round(cov_pct, 1),
            'goals': goal_count,
            'failures': fail_count,
            'map_size': [w, h],
            'resolution': m.info.resolution,
            'sim_time': round(self._now(), 1),
            'pgm': pgm,
            'yaml': yaml_path,
        }
        js = prefix + '.json'
        with open(js, 'w') as f:
            json.dump(summary, f, indent=2)
        self.get_logger().info(f'已保存: {pgm} / {yaml_path} / {js} (coverage {cov_pct:.1f}%)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--box-json', required=True)
    parser.add_argument('--box-id', type=int, required=True)
    parser.add_argument('--robot-name', default='go2_1')
    parser.add_argument('--action-server', default='/navigate_to_pose')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--save-prefix', default='explored_map')
    parser.add_argument('--max-goal-distance', type=float, default=15.0,
                        help='前沿搜索半径(米)。大分区用 15~30')
    parser.add_argument('--coverage-target', type=float, default=0.95,
                        help='覆盖率达标阈值。大分区很难到 95%%，可设 0.6~0.8')
    parser.add_argument('--auto-split', type=float, default=0.0,
                        help='>0: 把 box 切成约 N 米的小块，一块一块推进（大分区推荐 30）')
    args = parser.parse_args()

    with open(args.box_json, 'r') as f:
        data = json.load(f)

    box = None
    for b in data['boxes']:
        if b['id'] == args.box_id:
            box = b
            break

    if box is None:
        print(f'Box {args.box_id} not found')
        return

    rclpy.init()
    e = SimpleExplorer(args.robot_name, args.box_id, box, args.action_server, args.map_topic,
                       args.save_prefix, args.max_goal_distance, args.coverage_target,
                       args.auto_split)
    try:
        e.run()
    finally:
        # 中断(Ctrl-C/异常)也尽量把地图存下来
        try:
            if e._map is not None:
                e.get_logger().info('退出前保存地图...')
                e._save_result(e._coverage() * 100.0, -1, -1)
        except Exception as ex:
            print('保存失败:', ex)
    e.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
