source /opt/ros/humble/setup.bash
source install/setup.bash

alias run_lidar="ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/rplidar"
alias view_lidar="ros2 launch sllidar_ros2 view_sllidar_c1_launch.py serial_port:=/dev/rplidar"
alias run_agent='ros2 run micro_ros_agent micro_ros_agent serial -b 921600 --dev /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066BFF485270535067113035-if02' 
alias run_depth_cam='ros2 run astra_camera astra_launch.xml'
alias run='ros2 launch robot bringup.launch.py'
alias auto_run='ros2 launch robot bringup.launch.py slam:=true nav2:=true'
alias build='colcon build --symlink-install --packages-select robot'  
alias remove='rm -rf build/robot install/robot'
alias git_reset='git reset --hard HEAD && git clean -fd'
alias flash='make -j16 && st-flash --reset write build/*.bin 0x08000000'
alias rebuild_libmicroros='docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble'