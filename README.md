哨兵导航，采用北极熊框架


🤪️北极熊
cd ros_ws
source install/setup.bash
sudo chmod 777 /dev/ttyACM0

建图模式
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
保存栅格地图
ros2 run nav2_map_server map_saver_cli -f rmul2027 --ros-args -r map:=/map

导航模式
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=RMUL2026 \
slam:=False \
use_robot_state_pub:=True
