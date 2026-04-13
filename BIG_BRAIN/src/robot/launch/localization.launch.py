import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription  
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  
from launch.substitutions import PythonExpression

def generate_launch_description():  
    slam_config = os.path.join(get_package_share_directory('robot'), 'config', 'slam.yaml')
    ekf_config = os.path.join(get_package_share_directory('robot'), 'config', 'ekf.yaml')
    amcl_config = os.path.join(get_package_share_directory('robot'), 'config', 'amcl.yaml')
    map_path = os.path.join(get_package_share_directory('robot'), 'map', 'my_map.yaml')

    # ===== Static Map -> Odom =====
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',

        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('slam'), "' != 'true' and ",
                "'", LaunchConfiguration('amcl'), "' != 'true' and ",
                "'", LaunchConfiguration('nav2'), "' != 'true'"
            ])
        )
    )

    # ===== EKF (Sensor Fusion) =====
    imu_relay = Node(
        package='robot',
        executable='imu_relay',
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odom')] 
    )

    # ===== SLAM (Localization and Mapping) =====
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # ===== AMCL (Localization) =====
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            amcl_config,
            {'yaml_filename': map_path},
        ],
        condition=IfCondition(LaunchConfiguration('amcl'))
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
        ],
        condition=IfCondition(LaunchConfiguration('amcl'))
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=IfCondition(LaunchConfiguration('amcl'))
    )
    

    return LaunchDescription([
        imu_relay,
        ekf,
        map_server,
        amcl,
        lifecycle_manager_localization,
        slam_toolbox, 
        static_map_to_odom
    ])
