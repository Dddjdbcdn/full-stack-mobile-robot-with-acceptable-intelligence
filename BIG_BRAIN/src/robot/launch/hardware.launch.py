import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '-b', '921600',
                '--dev', '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066BFF485270535067113035-if02'],
        output='screen',
    )

    stm32_reset = ExecuteProcess(
        cmd=['st-flash', '--serial', '066BFF485270535067113035', 'reset'],
        output='screen',
    )

    delayed_stm32_reset = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=micro_ros_agent,
            on_start=[stm32_reset],
        )
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
        ),
         launch_arguments={'serial_port': '/dev/rplidar', 'frame_id': 'lidar'}.items(),
    )

    depth_camera_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('astra_camera'), 'launch', 'astra.launch.xml')
        ),
    )

    return LaunchDescription([
        micro_ros_agent,
        delayed_stm32_reset,
        lidar_node,
        depth_camera_node
    ])