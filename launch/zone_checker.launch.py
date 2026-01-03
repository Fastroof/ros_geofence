import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_geofence')

    # Аргументи
    geojson_file_arg = DeclareLaunchArgument(
        'geojson_file',
        default_value=os.path.join(pkg_share, 'config', 'zone.geojson'),
        description='Path to GeoJSON file with zone polygon'
    )
    costmap_resolution_arg = DeclareLaunchArgument(
        'costmap_resolution',
        default_value='0.1',
        description='Costmap resolution in meters'
    )
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='Topic for costmap publishing'
    )
    geofence_zone_topic_arg = DeclareLaunchArgument(
        'geofence_zone_topic',
        default_value='/geofence_zone',
        description='Topic for geofence zone polygon'
    )

    # Нода
    zone_checker_node = Node(
        package='ros_geofence',
        executable='zone_checker',
        name='zone_checker',
        parameters=[{
            'geojson_file': LaunchConfiguration('geojson_file'),
            'costmap_resolution': LaunchConfiguration('costmap_resolution'),
            'costmap_topic': LaunchConfiguration('costmap_topic'),
            'geofence_zone_topic': LaunchConfiguration('geofence_zone_topic'),
        }],
        output='screen'
    )

    return launch.LaunchDescription([
        geojson_file_arg,
        costmap_resolution_arg,
        costmap_topic_arg,
        geofence_zone_topic_arg,
        zone_checker_node,
    ])
