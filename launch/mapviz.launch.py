import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os
import tempfile
from ament_index_python.packages import get_package_share_directory


def generate_mapviz_config(context, *args, **kwargs):
    template_path = os.path.join(
        get_package_share_directory('ros_geofence'), "config", "mapviz.mvc"
    )
    with open(template_path, 'r') as f:
        config_content = f.read()
    
    api_key = LaunchConfiguration('maptiler_api_key').perform(context)
    config_content = config_content.replace('${MAPTILER_API_KEY}', api_key)

    config_dir = os.path.join(tempfile.gettempdir(), 'ros_geofence')
    os.makedirs(config_dir, exist_ok=True)
    config_path = os.path.join(config_dir, 'mapviz.mvc')
    
    with open(config_path, 'w') as f:
        f.write(config_content)
    
    return [
        Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": config_path}]
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'maptiler_api_key',
            default_value=EnvironmentVariable('MAPTILER_API_KEY', default_value=''),
            description='MapTiler API key for satellite tiles'
        ),
        OpaqueFunction(function=generate_mapviz_config),
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "gps/fix"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "map",
                "--child-frame-id", "origin"
            ]
        )
    ])
