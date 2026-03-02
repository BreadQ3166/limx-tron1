cd ~/limx_ws
colcon build --packages-select controller_msgs

必须先source一下，让系统识别到controller_msgs，再编译converter
source install/setup.bash
colcon build --packages-select imu_sync_converter

source install/setup.bash

ros2 launch robot_visualization pointfoot_plot_hw.launch

运行节点。假设 NUC发过来的 imustamp 单位是微秒，则 scale 为 1e-6 (0.000001)
如果是毫秒，设为 0.001。如果是纳秒，设为 0.000000001
ros2 run imu_sync_converter imu_converter_node --ros-args -p imustamp_scale:=0.000001 -p alpha:=0.05


