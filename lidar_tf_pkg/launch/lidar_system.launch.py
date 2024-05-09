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
        

    ])
