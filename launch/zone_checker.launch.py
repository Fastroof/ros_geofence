import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_geofence')

    geojson_file_arg = DeclareLaunchArgument(
        'geojson_file',
        default_value=os.path.join(pkg_share, 'config', 'zone.geojson'),
        description='Path to GeoJSON file with zone polygon'
    )
    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic',
        default_value='/gps/fix',
        description='GPS NavSatFix topic to subscribe'
    )

    zone_checker_node = Node(
        package='ros_geofence',
        executable='zone_checker',
        name='zone_checker',
        parameters=[{
            'geojson_file': LaunchConfiguration('geojson_file'),
            'gps_topic': LaunchConfiguration('gps_topic'),
        }],
        output='screen'
    )

    return launch.LaunchDescription([
        geojson_file_arg,
        gps_topic_arg,
        zone_checker_node,
    ])
