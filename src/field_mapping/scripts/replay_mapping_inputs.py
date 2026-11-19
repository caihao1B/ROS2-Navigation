#!/usr/bin/env python3
"""Publish a recorded sensor segment in an isolated mapping-only ROS domain."""
import io
import json
import os
import time
from pathlib import Path
from zipfile import ZipFile

import numpy as np
from PIL import Image as PillowImage
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def stamp(seconds):
    sec, nanosec = divmod(round(seconds * 1e9), 10**9)
    return Time(sec=sec, nanosec=nanosec)


def rotation(pose):
    x, y, z, w = pose['xyzw']
    return Quaternion(x=x, y=y, z=z, w=w)


def transform(header, child, pose):
    result = TransformStamped(header=header, child_frame_id=child)
    x, y, z = pose['xyz']
    result.transform.translation = Vector3(x=x, y=y, z=z)
    result.transform.rotation = rotation(pose)
    return result


def publish_sequence(node, archive):
    manifest = json.loads(archive.read('manifest.json'))
    if node.count_publishers('/clock'):
        raise RuntimeError('A simulation clock already exists. Choose an unused ROS domain.')
    clock = node.create_publisher(Clock, '/clock', 10)
    odom = node.create_publisher(Odometry, '/localization/odometry', 10)
    lidar = node.create_publisher(PointCloud2, '/sensors/lidar/points', qos_profile_sensor_data)
    transforms = TransformBroadcaster(node)
    calibration_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cameras = {}
    for side, intrinsics in manifest['cameras'].items():
        topic = '/sensors/stereo/' + side
        cameras[side] = (
            node.create_publisher(Image, topic + '/image_rect', qos_profile_sensor_data),
            node.create_publisher(CameraInfo, topic + '/camera_info', calibration_qos),
            CameraInfo(distortion_model='plumb_bob', **intrinsics),
        )
    # Wait for DDS discovery before emitting the short sequence.
    deadline = time.monotonic() + 10
    publishers = [lidar, odom]
    for image_pub, info_pub, _ in cameras.values():
        publishers.extend([image_pub, info_pub])
    while not all(pub.get_subscription_count() for pub in publishers):
        if time.monotonic() > deadline:
            raise RuntimeError('Start mapping.launch.py in the same ROS domain before replaying.')
        rclpy.spin_once(node, timeout_sec=0.1)

    for frame in manifest['frames']:
        if node.count_publishers('/clock') > 1:
            raise RuntimeError('Another simulation clock appeared. Replay stopped.')
        at = stamp(frame['t'])
        clock.publish(Clock(clock=at))
        pose = frame['odom_pose']
        header = Header(stamp=at, frame_id='odom')
        motion = Odometry(header=header, child_frame_id='base_link')
        x, y, z = pose['xyz']
        motion.pose.pose.position = Point(x=x, y=y, z=z)
        motion.pose.pose.orientation = rotation(pose)
        base = Header(stamp=at, frame_id='base_link')
        edges = [transform(header, 'base_link', pose)]
        for side, (image_pub, info_pub, calibration) in cameras.items():
            optical = side + '_optical'
            edges.append(transform(base, optical, frame['camera_poses'][side]))
            image_header = Header(stamp=at, frame_id=optical)
            calibration.header = image_header
            info_pub.publish(calibration)
            with PillowImage.open(io.BytesIO(archive.read(frame[side]))) as photo:
                rgb = photo.mode == 'RGB'
                image_pub.publish(Image(header=image_header, height=photo.height, width=photo.width,
                                        encoding='rgb8' if rgb else 'mono8',
                                        step=photo.width * (3 if rgb else 1), data=photo.tobytes()))
        transforms.sendTransform(edges)
        odom.publish(motion)
        points = np.load(io.BytesIO(archive.read(frame['scan'])), allow_pickle=False)
        lidar.publish(point_cloud2.create_cloud_xyz32(base, points))
        time.sleep(0.65)

    # Advance only the clock to service pending ROS timers after the last frame.
    for step in range(1, 11):
        clock.publish(Clock(clock=stamp(frame['t'] + 0.05 * step)))
        time.sleep(0.05)
    print('观测已发送。等建图终端处理完后按 Ctrl+C，使 RTAB-Map 保存数据库。')


def main():
    if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
        raise SystemExit('Set ROS_DOMAIN_ID to a dedicated nonzero domain in both terminals.')
    rclpy.init()
    node = Node('mapping_observation_replay')
    try:
        # Give DDS time to discover an existing clock before creating our own.
        time.sleep(1.0)
        sequence = Path(get_package_share_directory('field_mapping')) / 'example' / 'sequence.zip'
        with ZipFile(sequence) as archive:
            publish_sequence(node, archive)
    except RuntimeError as error:
        raise SystemExit(str(error)) from error
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
