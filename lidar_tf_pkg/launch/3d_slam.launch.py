import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'
        )
    )

    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'mapping.rviz'
        )
    )

    ldlidar_node = launch_ros.actions.Node(
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
    )

    scanmatcher_node = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[
            ('/input_cloud', '/lidar_cloud'),
            ('/imu', '/imu/data'),
        ],
        output='screen',
        arguments=['--ros-args', '--param', 'transform_timeout:=1.0']
    )

    static_transform_publisher_odom = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    graph_based_slam_node = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        remappings=[
            ('/map_array', '/map_array'),
            ('/modified_path', '/modified_path'),
            ('/modified_map', '/modified_map')
        ],
        output='screen'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir],
        output='screen'
    )

    rocking_motion_node = launch_ros.actions.Node(
        package='lidar_tf_pkg',
        executable='rocking_motion_node',
        name='rocking_motion_node',
        output='screen',
        parameters=[{'param_name': 'param_value'}]
    )

    lidar_3d_node = launch_ros.actions.Node(
        package='lidar_tf_pkg',
        executable='lidar_3d_node',
        name='lidar_3d_node',
        output='screen',
        parameters=[{'param_name': 'param_value'}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'
        ),
        DeclareLaunchArgument(
            'rviz_param_dir',
            default_value=rviz_param_dir,
            description='Full path to RViz parameter file to load'
        ),
        ldlidar_node,
        rocking_motion_node,
        lidar_3d_node,
        scanmatcher_node,
        static_transform_publisher_odom,
        graph_based_slam_node,
        rviz_node
    ])

