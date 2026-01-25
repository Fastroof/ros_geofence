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

    output_pgm_arg = DeclareLaunchArgument(
        'output_pgm',
        default_value=os.path.join(pkg_share, 'config', 'filter_mask.pgm'),
        description='Output PGM file path'
    )

    output_yaml_arg = DeclareLaunchArgument(
        'output_yaml',
        default_value=os.path.join(pkg_share, 'config', 'filter_mask.yaml'),
        description='Output YAML file path'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.5',
        description='Grid resolution in meters per cell'
    )

    padding_arg = DeclareLaunchArgument(
        'padding',
        default_value='50.0',
        description='Padding around zone in meters'
    )

    filter_mask_generator_node = Node(
        package='ros_geofence',
        executable='filter_mask_generator',
        name='filter_mask_generator',
        parameters=[{
            'geojson_file': LaunchConfiguration('geojson_file'),
            'output_pgm': LaunchConfiguration('output_pgm'),
            'output_yaml': LaunchConfiguration('output_yaml'),
            'resolution': LaunchConfiguration('resolution'),
            'padding': LaunchConfiguration('padding'),
        }],
        output='screen'
    )

    return launch.LaunchDescription([
        geojson_file_arg,
        output_pgm_arg,
        output_yaml_arg,
        resolution_arg,
        padding_arg,
        filter_mask_generator_node,
    ])
