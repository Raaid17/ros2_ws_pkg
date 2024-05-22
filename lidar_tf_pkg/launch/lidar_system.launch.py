from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_tf_pkg',
            executable='rocking_motion_node',
            name='rocking_motion_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}]  # Specific parameters
        ),
        Node(
            package='lidar_tf_pkg',
            executable='lidar_3d_node',
            name='lidar_3d_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}]  # Specific parameters
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                {'resolution': 0.05},
                {'frame_id': 'base_link'},
                {'sensor_model/max_range': 10.0},
            ],
            remappings=[
                ('cloud_in', '/lidar_cloud'),
            ]
        ),
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD19',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD19'},
                {'port_name': '/dev/ttyUSB0'},
                {'topic_name': 'scan'},
                {'frame_id': 'base_laser'},
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        ),
    ])

