#!/usr/bin/env python3
"""Synchronize observed stereo, lidar and odometry for RTAB-Map."""

from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rtabmap_msgs.msg import RGBDImage
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


def timestamp(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def rotate_translate(points, translation, quaternion):
    attitude = Rotation.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return attitude.apply(points) + [translation.x, translation.y, translation.z]


def stable_surfaces(points, odometry, history, stamp):
    pose = odometry.pose.pose
    world = rotate_translate(points, pose.position, pose.orientation)
    support = np.zeros(len(points), dtype=np.int8)
    for previous_time, previous_tree in history:
        if 0.15 < stamp - previous_time < 3.0:
            support += previous_tree.query(world, k=1)[0] < 0.25
    history.append((stamp, cKDTree(world)))
    return points[(points[:, 2] < 0.18) | (support >= 2)]


class MappingInputs(Node):
    def __init__(self):
        super().__init__('mapping_inputs')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.vehicles_bounds = self.declare_parameter(
            'self_filter_bounds', [-0.65, 1.5, -0.65, 0.65, 1.0]).value
        self.frames = {side: deque(maxlen=60) for side in ('left', 'right')}
        self.calibration = {}
        self.clouds = deque(maxlen=30)
        self.poses = deque(maxlen=150)
        self.history = deque(maxlen=5)
        self.last_processed = float('-inf')
        self.previous_cloud = None
        self.tf_buffer = Buffer(node=self)
        self.listener = TransformListener(self.tf_buffer, self)
        self.scan_pub = self.create_publisher(PointCloud2, '/mapping/input/cloud', qos_profile_sensor_data)
        self.stereo_pub = self.create_publisher(RGBDImage, '/mapping/input/stereo', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(Odometry, '/mapping/input/odom', 10)
        self.create_subscription(PointCloud2, '/sensors/lidar/points', self.add_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odometry', lambda msg: self.poses.append(msg), 50)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def image_callback(frames):
            def enqueue(image):
                frames.append(image)
            return enqueue

        def calibration_callback(side):
            def save_info(info):
                self.calibration[side] = info
            return save_info

        for side, frames in self.frames.items():
            prefix = f'/sensors/stereo/{side}'
            self.create_subscription(Image, prefix + '/image_rect', image_callback(frames), qos_profile_sensor_data)
            # Bridge cameras may stream calibration or latch it once.
            for qos in (latched, qos_profile_sensor_data):
                self.create_subscription(CameraInfo, prefix + '/camera_info', calibration_callback(side), qos)
        self.create_timer(0.025, self.process)

    def add_cloud(self, cloud):
        now = timestamp(cloud)
        if self.previous_cloud is not None and now < self.previous_cloud:
            self.clouds.clear()
            self.history.clear()
            self.poses.clear()
            for frames in self.frames.values():
                frames.clear()
            self.last_processed = float('-inf')
        self.previous_cloud = now
        self.clouds.append(cloud)

    def process(self):
        if len(self.calibration) != 2 or not self.poses or not all(self.frames.values()):
            return
        while self.clouds:
            cloud = self.clouds[0]
            stamp = timestamp(cloud)
            if stamp < 5.0 or stamp - self.last_processed < 0.5 - 1e-4:
                self.clouds.popleft()
                continue
            if (timestamp(self.poses[-1]) < stamp + 0.08 or
                    min(timestamp(frames[-1]) for frames in self.frames.values()) < stamp + 0.07):
                return
            self.clouds.popleft()
            right_at = {timestamp(frame): frame for frame in self.frames['right']}
            pairs = [frame for frame in self.frames['left'] if timestamp(frame) in right_at]
            if not pairs:
                continue
            left = min(pairs, key=lambda frame: abs(timestamp(frame) - stamp))
            odom = min(self.poses, key=lambda frame: abs(timestamp(frame) - stamp))
            if max(abs(timestamp(left) - stamp), abs(timestamp(odom) - stamp)) > 0.04:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, cloud.header.frame_id, Time.from_msg(cloud.header.stamp)).transform
            except TransformException:
                continue
            self.last_processed = stamp
            points = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z']).reshape(-1, 3)
            distance = np.linalg.norm(points, axis=1)
            points = points[np.isfinite(points).all(axis=1) & (distance > 1) & (distance < 35)]
            if not len(points):
                continue
            points = rotate_translate(points, transform.translation, transform.rotation)
            x0, x1, y0, y1, roof = self.vehicles_bounds
            inside = ((points[:, 0] > x0) & (points[:, 0] < x1) &
                      (points[:, 1] > y0) & (points[:, 1] < y1) & (points[:, 2] < roof))
            points = points[~inside]
            if not len(points):
                continue
            _, indices = np.unique(np.floor(points / 0.18).astype(np.int32), axis=0, return_index=True)
            points = points[np.sort(indices)]
            if len(points) > 6000:
                points = points[np.linspace(0, len(points) - 1, 6000, dtype=int)]
            filtered = stable_surfaces(points, odom, self.history, stamp)
            if len(self.history) < 3 or np.count_nonzero(filtered[:, 2] > 0.18) < 100:
                continue
            packet = RGBDImage()
            packet.header = left.header
            packet.rgb, packet.depth = left, right_at[timestamp(left)]
            packet.rgb_camera_info = self.calibration['left']
            packet.depth_camera_info = self.calibration['right']
            header = Header(stamp=cloud.header.stamp, frame_id=self.base_frame)
            self.scan_pub.publish(point_cloud2.create_cloud_xyz32(header, filtered.astype('float32')))
            self.stereo_pub.publish(packet)
            self.pose_pub.publish(odom)


def main():
    rclpy.init()
    node = MappingInputs()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
