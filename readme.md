构建包:
colcon build --packages-select tactile_sensor_ros2

加载环境：
source install/setup.bash

运行：
ros2 launch tactile_sensor_ros2 tactile_sensor_launch.py


# 查看话题列表
ros2 topic list

# 实时查看传感器数据
ros2 topic echo /tactile_sensor/finger_0/data
ros2 topic echo /tactile_sensor/finger_0/wrench

# 查看节点状态
ros2 node list
ros2 node info /tactile_sensor_node
