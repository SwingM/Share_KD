#!/usr/bin/env python3
"""
Model-states odom bridge — bypasses Gazebo p3d namespace-bleed bug.

Reads /gazebo/model_states, finds the dog model by name pattern,
publishes /{ns}/odom + TF {ns}/odom -> {ns}/base_link.

Usage:
  python3 gt_odom_bridge_modelstates.py --ros-args -p namespace:=go2_1
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from gazebo_msgs.msg import ModelStates
import math


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


class ModelStatesOdomBridge(Node):
    def __init__(self):
        super().__init__('gt_odom_bridge_modelstates')
        self.declare_parameter('namespace', 'go2_1')
        self.declare_parameter('model_name', '')
        ns = self.get_parameter('namespace').value
        model_name = self.get_parameter('model_name').value
        if not model_name:
            model_name = f'{ns}_gazebo'
        self.model_name = model_name
        self.ns = ns
        self.odom_frame = f'{ns}/odom'
        self.child_frame = f'{ns}/base_link'
        self.last_stamp = None

        be = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.cb, be)
        self.odom_pub = self.create_publisher(Odometry, f'/{ns}/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info(
            f'model_states bridge: model={model_name} -> /{ns}/odom')

    def cb(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        ox = pose.position.x
        oy = pose.position.y
        oyaw = yaw_from_quat(pose.orientation)

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat_from_yaw(oyaw)
        odom.twist.twist.linear.x = twist.linear.x
        odom.twist.twist.linear.y = twist.linear.y
        odom.twist.twist.angular.z = twist.angular.z
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = ox
        t.transform.translation.y = oy
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = ModelStatesOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
