import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml
import pyproj
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from ament_index_python.packages import get_package_share_directory


def adjust_mask_and_create_nodes(context):
    """
    OpaqueFunction to adjust mask origin based on GPS fix from /gps/fix topic.
    """
    # Get launch arguments
    mask_yaml = LaunchConfiguration('mask_yaml').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    filter_type = LaunchConfiguration('filter_type').perform(context)
    gps_topic = LaunchConfiguration('gps_topic').perform(context)
    timeout = float(LaunchConfiguration('gps_timeout').perform(context))
    
    adjusted_yaml = '/tmp/filter_mask_adjusted.yaml'
    
    print(f"[costmap_filter] Waiting for GPS fix on {gps_topic}...")
    print(f"[costmap_filter] Input mask: {mask_yaml}")
    print(f"[costmap_filter] Output mask: {adjusted_yaml}")
    
    # Initialize rclpy if not already initialized
    if not rclpy.ok():
        rclpy.init()
    
    # Create temporary node to receive GPS fix
    class GpsReceiver(RclpyNode):
        def __init__(self):
            super().__init__('gps_receiver_temp')
            self.datum_lat = None
            self.datum_lon = None
            self.gps_received = False
            
            qos = QoSProfile(
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
            
            self.sub = self.create_subscription(
                NavSatFix,
                gps_topic,
                self.gps_callback,
                qos
            )
        
        def gps_callback(self, msg):
            if not self.gps_received and msg.status.status >= 0:  # Valid fix
                self.datum_lat = msg.latitude
                self.datum_lon = msg.longitude
                self.gps_received = True
                self.get_logger().info(f'Received GPS fix: {self.datum_lat:.6f}, {self.datum_lon:.6f}')
    
    # Wait for GPS fix
    node = GpsReceiver()
    import time
    start_time = time.time()
    
    while not node.gps_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > timeout:
            node.get_logger().error(f'Timeout waiting for GPS fix after {timeout}s')
            node.destroy_node()
            raise RuntimeError('GPS fix timeout')
    
    if not node.gps_received:
        node.destroy_node()
        raise RuntimeError('Failed to receive GPS fix')
    
    datum_lat = node.datum_lat
    datum_lon = node.datum_lon
    node.destroy_node()
    
    print(f"[costmap_filter] GPS datum: {datum_lat:.6f}, {datum_lon:.6f}")
    
    # Read mask YAML
    with open(mask_yaml, 'r') as f:
        mask_config = yaml.safe_load(f)
    
    old_origin = mask_config['origin']
    print(f"[costmap_filter] Old origin (UTM): [{old_origin[0]:.2f}, {old_origin[1]:.2f}, {old_origin[2]:.2f}]")
    
    # Determine UTM zone and convert datum to UTM
    zone_number = int((datum_lon + 180.0) / 6.0) + 1
    is_northern = datum_lat >= 0
    epsg_code = 32600 + zone_number if is_northern else 32700 + zone_number
    utm_crs = f'EPSG:{epsg_code}'
    
    transformer = pyproj.Transformer.from_crs('EPSG:4326', utm_crs, always_xy=True)
    datum_utm_x, datum_utm_y = transformer.transform(datum_lon, datum_lat)
    
    hemisphere = 'N' if is_northern else 'S'
    print(f"[costmap_filter] UTM zone: {zone_number}{hemisphere} ({utm_crs})")
    print(f"[costmap_filter] Datum UTM: x={datum_utm_x:.2f}, y={datum_utm_y:.2f}")
    
    # Calculate new origin
    new_origin = [
        old_origin[0] - datum_utm_x,
        old_origin[1] - datum_utm_y,
        0.0
    ]
    print(f"[costmap_filter] New origin (local): [{new_origin[0]:.2f}, {new_origin[1]:.2f}, {new_origin[2]:.2f}]")
    
    # Update config
    mask_config['origin'] = new_origin
    
    # Ensure frame_id is set to 'map' for proper TF integration
    if 'frame_id' not in mask_config:
        mask_config['frame_id'] = 'map'
        print(f"[costmap_filter] Added frame_id: map")
    
    # Write adjusted YAML
    with open(adjusted_yaml, 'w') as f:
        yaml.dump(mask_config, f, default_flow_style=False)
    
    print(f"[costmap_filter] ✓ Mask origin adjusted successfully!")
    
    # Create and return nodes with adjusted mask
    use_sim_time_bool = use_sim_time.lower() == 'true'
    
    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'yaml_filename': adjusted_yaml,  # Use adjusted mask!
            'use_sim_time': use_sim_time_bool
        }],
        remappings=[('map', '/global_costmap/filter_mask')]
    )
    
    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time_bool,
            'type': int(filter_type),
            'filter_info_topic': '/global_costmap/costmap_filter_info',
            'mask_topic': '/global_costmap/filter_mask',
            'base': 0.0,
            'multiplier': 1.0
        }]
    )
    
    lifecycle_manager_filters = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_filters',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time_bool,
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server']
        }]
    )
    
    return [filter_mask_server, costmap_filter_info_server, lifecycle_manager_filters]


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_geofence')

    # Declare arguments
    mask_yaml_arg = DeclareLaunchArgument(
        'mask_yaml',
        default_value=os.path.join(pkg_share, 'config', 'filter_mask.yaml'),
        description='Path to base filter mask YAML file (with UTM coordinates)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='0',
        description='Filter type: 0 = Keepout, 1 = Speed Limit'
    )
    
    gps_topic_arg = DeclareLaunchArgument(
        'gps_topic',
        default_value='/gps/fix',
        description='Topic to receive GPS fix for datum'
    )
    
    gps_timeout_arg = DeclareLaunchArgument(
        'gps_timeout',
        default_value='300.0',
        description='Timeout in seconds to wait for GPS fix'
    )

    # OpaqueFunction to adjust mask and create nodes
    adjust_and_launch = OpaqueFunction(function=adjust_mask_and_create_nodes)

    return launch.LaunchDescription([
        mask_yaml_arg,
        use_sim_time_arg,
        filter_type_arg,
        gps_topic_arg,
        gps_timeout_arg,
        adjust_and_launch,
    ])
