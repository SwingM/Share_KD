#!/usr/bin/env python3
"""fall_rescue.py — 翻倒自动救援监看（仿真扶正式）

每只狗一个进程，与 explore_simple.py 并行运行，两者不直接耦合。

监测输入
  /<robot>/imu                 IMU 姿态（roll/pitch）—— 用户要求的传感器判据
  /<robot>/odom/ground_truth   世界真值位姿 x,y,z,yaw（扶正时保持 x,y,yaw）

计时
  一律用真实时钟（墙钟）。翻倒检测阈值 fall_dwell 就是真实秒数：
  Gazebo 3 狗 RTF≈0.1，所以真实 10s ≈ 1 仿真秒——这是刻意选择，方便及时救援；
  （注意：导航超时等判据仍应使用仿真时钟，那个结论不适用于此处。）

判定
  狗至少"站直站稳"过一次（armed）之后，若 |roll| 或 |pitch| 超过阈值并
  【持续 fall_dwell 真实秒】，即判定为翻倒（倒置/侧翻）。

救援（原地，保持当前 x,y 与朝向）
  1) 写 'P' 到 keys fifo        -> rl_sim 进 Passive（卸掉策略输出，仅阻尼）
  2) /gazebo/set_entity_state    -> 把模型扶正到 (x, y, rescue_z, roll=0,pitch=0,yaw)
  3) 被阻尼放平(低且水平)         -> 写 '0' 触发 GetUp
  4) 重新站稳                     -> 写 '1' 确保进入 RL 策略
  5) 回到监测；explore_simple.py 会自然继续发目标，任务续跑

用法
  python3 fall_rescue.py --robot go2_1 --fifo .log/keys_1
"""
import argparse
import errno
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, Quaternion, Twist


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _roll_pitch_from_quat(q):
    """四元数 -> (roll_deg, pitch_deg)。与 explore_simple.py / rl_sim 口径一致。"""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return math.degrees(roll), math.degrees(pitch)


class FallRescue(Node):
    MONITOR = 'MONITOR'
    RESCUE = 'RESCUE'

    def __init__(self, args):
        super().__init__(f'fall_rescue_{args.robot}')
        self.robot = args.robot
        self.model = args.model
        self.fifo = args.fifo
        self.fall_roll = args.fall_roll_deg
        self.fall_pitch = args.fall_pitch_deg
        self.fall_dwell = args.fall_dwell
        self.rescue_z = args.rescue_z
        self.arm_stand_z = args.arm_stand_z
        self.rest_z = args.rest_z
        self.level_deg = args.level_deg
        self.arm_dwell = args.arm_dwell
        self.rest_settle = args.rest_settle
        self.settle = args.settle
        self.step_timeout = args.step_timeout

        # --- 传感器 ---
        self._imu_rp = None    # (roll_deg, pitch_deg)
        self._odom_rp = None
        self._pose = None      # (x, y, yaw, z)

        # --- 判定状态 ---
        self._armed = False
        self._arm_since = None
        self._fallen_since = None

        # --- 救援状态机 ---
        self.state = self.MONITOR
        self._sub = None
        self._sub_since = 0.0
        self._deadline = 0.0
        self._teleport_future = None
        self._phase = None
        self._phase_since = 0.0

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Imu, f'/{self.robot}/imu', self._imu_cb, reliable)
        self.create_subscription(Odometry, f'/{self.robot}/odom/ground_truth',
                                 self._odom_cb, reliable)

        self._set_state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'fall_rescue 就绪: robot={self.robot} model={self.model} '
            f'判据=|roll|>{self.fall_roll}° 或 |pitch|>{self.fall_pitch}° 持续 '
            f'{self.fall_dwell}s(真实时钟); 救援=扶正到当前(x,y)/yaw, z={self.rescue_z}m '
            f'(站立阈值 z>{self.arm_stand_z}m, 放平阈值 z<{self.rest_z}m)')

    # ------------------------------------------------------------------ 回调
    def _imu_cb(self, msg):
        self._imu_rp = _roll_pitch_from_quat(msg.orientation)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._odom_rp = _roll_pitch_from_quat(q)
        self._pose = (p.x, p.y, _yaw_from_quat(q), p.z)

    def _now(self):
        """真实时钟（墙钟）。翻倒时长按真实秒计——Gazebo RTF≈0.1 时 10 真实秒
        ≈ 1 仿真秒，正是为了让救援及时发生。"""
        return time.time()

    # ------------------------------------------------------------------ 工具
    def _rp(self):
        """姿态优先用 IMU；IMU 未到时退回真值里程计。"""
        return self._imu_rp if self._imu_rp is not None else self._odom_rp

    def _level(self):
        rp = self._rp()
        return rp is not None and abs(rp[0]) < self.level_deg and abs(rp[1]) < self.level_deg

    def _standing(self):
        return (self._pose is not None and self._pose[3] > self.arm_stand_z
                and self._level())

    def _prone(self):
        return (self._pose is not None and 0.0 < self._pose[3] < self.rest_z
                and self._level())

    def _fallen(self):
        rp = self._rp()
        if rp is None:
            return False
        return abs(rp[0]) > self.fall_roll or abs(rp[1]) > self.fall_pitch

    def _send_key(self, ch):
        """往 rl_sim 的 keys fifo 写一个字节。无 reader 时最多重试 1s。"""
        end = time.time() + 1.0
        while time.time() < end:
            try:
                fd = os.open(self.fifo, os.O_WRONLY | os.O_NONBLOCK)
            except OSError as e:
                if e.errno in (errno.ENXIO, errno.ENOENT):
                    time.sleep(0.05)
                    continue
                self.get_logger().warn(f'[RESCUE] 打开 keys fifo 失败: {e}')
                return False
            try:
                os.write(fd, ch.encode())
            except OSError as e:
                self.get_logger().warn(f'[RESCUE] 写 keys fifo 失败: {e}')
                return False
            finally:
                os.close(fd)
            return True
        self.get_logger().warn(f'[RESCUE] 写 keys fifo 超时(无 reader): {self.fifo}')
        return False

    def _phase_ok(self, name, now):
        """子阶段内的持续判定（防抖）。"""
        if self._phase != name:
            self._phase = name
            self._phase_since = now
            return False
        return (now - self._phase_since) >= self.rest_settle

    def _clear_phase(self):
        self._phase = None

    # ------------------------------------------------------------------ 主循环
    def _tick(self):
        try:
            if self.state == self.MONITOR:
                self._tick_monitor()
            else:
                self._tick_rescue()
        except Exception as e:  # 任何异常都不该让监看线程死掉
            self.get_logger().warn(f'[RESCUE] tick 异常: {e}')

    def _tick_monitor(self):
        if self._rp() is None:
            return
        now = self._now()

        # 先确认狗确实站起来过，避免启动阶段(世界复位/趴伏/起身中)误判
        if self._standing():
            if self._arm_since is None:
                self._arm_since = now
            elif not self._armed and (now - self._arm_since) >= self.arm_dwell:
                self._armed = True
                rp = self._rp()
                self.get_logger().info(
                    f'[RESCUE] {self.robot} 已站稳，翻倒监测进入武装状态 '
                    f'(roll={rp[0]:.1f}° pitch={rp[1]:.1f}°)')
        else:
            self._arm_since = None

        if not self._fallen():
            if self._fallen_since is not None:
                self.get_logger().info('[RESCUE] 姿态已恢复，取消翻倒计时')
                self._fallen_since = None
            return

        if not self._armed:
            return

        rp = self._rp()
        if self._fallen_since is None:
            self._fallen_since = now
            self.get_logger().warn(
                f'[RESCUE] {self.robot} 疑似翻倒 (roll={rp[0]:.1f}° pitch={rp[1]:.1f}°), '
                f'持续 {self.fall_dwell}s(真实时钟)后执行原地救援')
            return

        if (now - self._fallen_since) >= self.fall_dwell:
            self._begin_rescue()

    # ------------------------------------------------------------------ 救援
    def _begin_rescue(self):
        self.state = self.RESCUE
        self._sub = 'P_SENT'
        self._sub_since = self._now()
        self._deadline = time.time() + self.step_timeout
        self._fallen_since = None
        self._teleport_future = None
        self._clear_phase()

        pose = self._pose
        if pose is not None:
            self.get_logger().error(
                f'[RESCUE] {self.robot} 翻倒超时，开始原地救援: '
                f'pos=({pose[0]:.2f},{pose[1]:.2f}) yaw={math.degrees(pose[2]):.1f}° z={pose[3]:.2f}m')
        else:
            self.get_logger().error(
                f'[RESCUE] {self.robot} 翻倒超时，但未收到真值位姿，将仅尝试起身')
        if not self._send_key('P'):
            self.get_logger().warn('[RESCUE] 发送 P(Passive) 失败')

    def _do_teleport(self):
        if self._pose is None:
            self._teleport_future = None
            self.get_logger().warn('[RESCUE] 无真值位姿，跳过扶正')
            return
        if not self._set_state_cli.wait_for_service(timeout_sec=2.0):
            self._teleport_future = None
            self.get_logger().warn('[RESCUE] /gazebo/set_entity_state 不可用，跳过扶正')
            return

        x, y, yaw, _z = self._pose
        st = EntityState()
        st.name = self.model
        st.pose.position = Point(x=float(x), y=float(y), z=float(self.rescue_z))
        st.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))
        st.twist = Twist()
        st.reference_frame = 'world'
        req = SetEntityState.Request()
        req.state = st
        self._teleport_future = self._set_state_cli.call_async(req)

    def _tick_rescue(self):
        now = self._now()

        if time.time() > self._deadline:
            self.get_logger().warn(
                f'[RESCUE] {self.robot} 救援步骤 {self._sub} 超时，放弃本轮 '
                f'(若仍翻倒会重新计时再试)')
            self._finish_rescue()
            return

        if self._sub == 'P_SENT':
            # 给 Passive 一点时间接管，再扶正（此时掉落有阻尼，落地更平整）
            if (now - self._sub_since) >= 1.0:
                self._do_teleport()
                self._sub = 'TELEPORT'
                self._sub_since = now
            return

        if self._sub == 'TELEPORT':
            if self._teleport_future is None:
                self._sub = 'WAIT_PRONE'
                self._sub_since = now
                return
            if not self._teleport_future.done():
                return
            try:
                resp = self._teleport_future.result()
                ok = bool(resp and resp.success)
            except Exception as e:
                ok = False
                self.get_logger().warn(f'[RESCUE] set_entity_state 异常: {e}')
            if ok:
                self.get_logger().info('[RESCUE] 已在当前坐标扶正，等待被动阻尼放平')
            else:
                self.get_logger().warn('[RESCUE] set_entity_state 返回失败，继续尝试起身')
            self._sub = 'WAIT_PRONE'
            self._sub_since = now
            return

        if self._sub == 'WAIT_PRONE':
            if self._prone():
                if self._phase_ok('prone', now):
                    self.get_logger().info('[RESCUE] 已放平，发送 0 触发 GetUp')
                    self._send_key('0')
                    self._sub = 'WAIT_STAND'
                    self._sub_since = now
                    self._clear_phase()
            elif self._standing():
                # auto-stand 可能已经自己完成起身，无需再发 0
                if self._phase_ok('stand', now):
                    self.get_logger().info('[RESCUE] 狗已自动站起，无需发送 0')
                    self._sub = 'WAIT_STAND'
                    self._sub_since = now
                    self._clear_phase()
            else:
                self._clear_phase()
            return

        if self._sub == 'WAIT_STAND':
            if self._standing():
                if self._phase_ok('ok', now):
                    self._send_key('1')
                    self.get_logger().info('[RESCUE] 已重新站稳，切回 RL 策略')
                    self._sub = 'SETTLE'
                    self._sub_since = now
                    self._clear_phase()
            else:
                self._clear_phase()
            return

        if self._sub == 'SETTLE':
            if (now - self._sub_since) >= self.settle:
                pose = self._pose
                where = f'({pose[0]:.2f},{pose[1]:.2f})' if pose else '?'
                self.get_logger().info(f'[RESCUE] 救援完成 {where}，恢复任务')
                self._finish_rescue()
            return

    def _finish_rescue(self):
        self.state = self.MONITOR
        self._sub = None
        self._phase = None
        self._teleport_future = None
        self._arm_since = None
        # 仍在翻倒则重新计时再试，避免卡死在"只救一次"
        self._fallen_since = self._now() if self._fallen() else None


def main():
    parser = argparse.ArgumentParser(description='翻倒自动救援监看（仿真扶正式）')
    parser.add_argument('--robot', required=True, help='机器人名，如 go2_1')
    parser.add_argument('--fifo', required=True, help='rl_sim 的 keys fifo 路径')
    parser.add_argument('--model', default=None, help='Gazebo 模型名，默认 <robot>_gazebo')
    parser.add_argument('--fall-roll-deg', type=float, default=75.0)
    parser.add_argument('--fall-pitch-deg', type=float, default=75.0)
    parser.add_argument('--fall-dwell', type=float, default=10.0,
                        help='翻倒需持续的真实秒数(墙钟)，默认 10s')
    parser.add_argument('--rescue-z', type=float, default=0.45, help='扶正后的基座高度(m)')
    parser.add_argument('--arm-stand-z', type=float, default=0.30, help='判定站立的最小基座高度(m)')
    parser.add_argument('--rest-z', type=float, default=0.20, help='判定已放平的最大基座高度(m)')
    parser.add_argument('--level-deg', type=float, default=40.0, help='判定"水平"的姿态阈值(度)')
    parser.add_argument('--arm-dwell', type=float, default=1.5, help='确认站稳所需真实秒数')
    parser.add_argument('--rest-settle', type=float, default=1.0, help='放平/站稳的防抖真实秒数')
    parser.add_argument('--settle', type=float, default=2.0, help='救援后确认稳定的真实秒数')
    parser.add_argument('--step-timeout', type=float, default=240.0, help='单步救援墙钟超时(s)')
    args = parser.parse_args()
    if args.model is None:
        args.model = f'{args.robot}_gazebo'

    rclpy.init()
    node = FallRescue(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
