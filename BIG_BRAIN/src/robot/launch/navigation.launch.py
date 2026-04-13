import os
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription  
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch_ros.actions import Node, LifecycleNode 

def generate_launch_description():
    nav2_config = os.path.join(get_package_share_directory('robot'), 'config', 'nav2.yaml')
    map_path = os.path.join(get_package_share_directory('robot'), 'map', 'my_map.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_config,
            'autostart': 'true',
        }.items(),
    )

    collision_monitor_node = LifecycleNode(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        namespace='',
        parameters=[nav2_config] 
    )

    collision_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['collision_monitor']}
        ]
    )

    return LaunchDescription([
        nav2_launch,
        collision_monitor_node,
        collision_lifecycle_manager
    ])