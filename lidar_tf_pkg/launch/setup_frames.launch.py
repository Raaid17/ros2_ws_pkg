from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_frame', 'lidar_frame'],
            # X, Y, Z, Roll, Pitch, Yaw, Parent_frame, Child_frame
        ),
        # You can add other nodes here as needed
    ])
