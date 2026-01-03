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

    # Аргументи для вибору компонентів
    use_fake_robot_arg = DeclareLaunchArgument(
        'use_fake_robot',
        default_value='true',
        description='Launch fake robot node'
    )
    use_zone_checker_arg = DeclareLaunchArgument(
        'use_zone_checker',
        default_value='true',
        description='Launch zone checker node'
    )
    use_mapviz_arg = DeclareLaunchArgument(
        'use_mapviz',
        default_value='true',
        description='Launch mapviz'
    )

    # Аргументи для fake_robot
    latitude_arg = DeclareLaunchArgument(
        'latitude',
        default_value='47.254',
        description='Robot latitude'
    )
    longitude_arg = DeclareLaunchArgument(
        'longitude',
        default_value='30.44',
        description='Robot longitude'
    )

    # Аргументи для zone_checker
    geojson_file_arg = DeclareLaunchArgument(
        'geojson_file',
        default_value=os.path.join(pkg_share, 'config', 'zone.geojson'),
        description='Path to GeoJSON file'
    )

    # Аргумент для MapTiler API ключа
    maptiler_api_key_arg = DeclareLaunchArgument(
        'maptiler_api_key',
        default_value=EnvironmentVariable('MAPTILER_API_KEY', default_value=''),
        description='MapTiler API key for satellite tiles'
    )

    # Fake robot
    fake_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'fake_robot.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_fake_robot')),
        launch_arguments={
            'latitude': LaunchConfiguration('latitude'),
            'longitude': LaunchConfiguration('longitude'),
        }.items()
    )

    # Zone checker
    zone_checker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'zone_checker.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_zone_checker')),
        launch_arguments={
            'geojson_file': LaunchConfiguration('geojson_file'),
        }.items()
    )

    # Mapviz
    mapviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_mapviz')),
        launch_arguments={
            'maptiler_api_key': LaunchConfiguration('maptiler_api_key'),
        }.items()
    )

    return launch.LaunchDescription([
        # Аргументи вибору компонентів
        use_fake_robot_arg,
        use_zone_checker_arg,
        use_mapviz_arg,
        # Аргументи fake_robot
        latitude_arg,
        longitude_arg,
        # Аргументи zone_checker
        geojson_file_arg,
        # Аргумент API ключа
        maptiler_api_key_arg,
        # Launch файли
        fake_robot,
        zone_checker,
        mapviz,
    ])
