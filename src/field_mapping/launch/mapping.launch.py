"""Build an observed map from stereo pairs, lidar and fused odometry."""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.logging import get_logging_directory


def generate_launch_description():
    config = Path(get_package_share_directory('field_mapping')) / 'config' / 'mapping.yaml'
    # Unique per launch. Opening an explicit existing database resumes that map.
    database = get_logging_directory().parent / ('farm_map_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.db')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    map_tf = ParameterValue(LaunchConfiguration('publish_map_tf'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('database', default_value=str(database)),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_map_tf', default_value='false',
                              description='Enable only if RTAB-Map owns map to odom.'),
        Node(package='field_mapping', executable='mapping_inputs.py', output='screen',
             parameters=[{'use_sim_time': sim_time}]),
        Node(package='rtabmap_slam', executable='rtabmap', namespace='mapping/rtabmap',
             name='mapper', output='screen', parameters=[str(config), {
                 'database_path': LaunchConfiguration('database'), 'publish_tf': map_tf,
                 'use_sim_time': sim_time}],
             remappings=[('rgbd_image', '/mapping/input/stereo'), ('scan_cloud', '/mapping/input/cloud'),
                         ('odom', '/mapping/input/odom')]),
    ])
