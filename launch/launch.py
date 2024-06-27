import os
import launch_ros
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def get_car_id_from_hostname():
    car_id = ''
    hostname = socket.gethostname()
    firstdigit = 1
    for char in hostname:
        if char.isdigit():
            if firstdigit == 0:
                car_id += char
            if firstdigit == 1 and char != "0":
                car_id += char
                firstdigit = 0
    return car_id

def generate_launch_description():
    car_id = get_car_id_from_hostname()
    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_description').find('car_description')
    default_model_path = os.path.join(pkg_share, 'urdf/car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/cost_map_rviz_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('visual_only', default_value='true'),
        DeclareLaunchArgument('frame', default_value='base_link', description='The fixed frame to be used in RViz'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model'), ' visual_only:=', LaunchConfiguration('visual_only')]), value_type=str)},]
        ),
        
        # Ego localization node
        Node(
            package='ego_localization',
            executable='ego_localization',
            name='ego_localization',
            output='screen'
        ),

        # Static transform publisher for map to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_base_link',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link']
        ),
        
        # Static transform publisher for base_link to origin_laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_origin_laser_frame',
            output='screen',
            arguments=['0.398', '0', '0.163', '0', '0', '0.5', '0.866', 'base_link', 'origin_laser_frame']
        ),

        # Static transform publisher for camera_link to origin_laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_camera_link_origin_laser_frame',
            output='screen',
            arguments=['0.031', '0.011', '-0.027', '0.000', '0.000', '0.500', '0.866', 'camera_link', 'origin_laser_frame']
        ),

        # lidar_camera_sf node
        Node(
            package='lidar_camera_sf',
            executable='lidar_camera_sf',
            name='lidar_camera_sf',
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])

if __name__ == '__main__':
    generate_launch_description()