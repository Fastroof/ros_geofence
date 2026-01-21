import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_geofence')
    launch_dir = os.path.join(pkg_share, 'launch')

    geojson_file_arg = DeclareLaunchArgument(
        'geojson_file',
        default_value=os.path.join(pkg_share, 'config', 'zone.geojson'),
        description='Path to GeoJSON file'
    )

    maptiler_api_key_arg = DeclareLaunchArgument(
        'maptiler_api_key',
        default_value=EnvironmentVariable('MAPTILER_API_KEY', default_value=''),
        description='MapTiler API key for satellite tiles'
    )

    fake_robot = Node(
        package='ros_geofence',
        executable='fake_robot',
        name='fake_robot',
        output='screen'
    )

    zone_checker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'zone_checker.launch.py')),
        launch_arguments={
            'geojson_file': LaunchConfiguration('geojson_file'),
        }.items()
    )

    mapviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapviz.launch.py')),
        launch_arguments={
            'maptiler_api_key': LaunchConfiguration('maptiler_api_key'),
        }.items()
    )

    return launch.LaunchDescription([
        geojson_file_arg,
        maptiler_api_key_arg,
        fake_robot,
        zone_checker,
        mapviz,
    ])
