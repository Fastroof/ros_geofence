import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Аргументи
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
    altitude_arg = DeclareLaunchArgument(
        'altitude',
        default_value='100.0',
        description='Robot altitude'
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='GPS publish rate in Hz'
    )

    # Нода
    fake_robot_node = Node(
        package='ros_geofence',
        executable='fake_robot',
        name='fake_robot',
        parameters=[{
            'latitude': LaunchConfiguration('latitude'),
            'longitude': LaunchConfiguration('longitude'),
            'altitude': LaunchConfiguration('altitude'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen'
    )

    return launch.LaunchDescription([
        latitude_arg,
        longitude_arg,
        altitude_arg,
        publish_rate_arg,
        fake_robot_node,
    ])
