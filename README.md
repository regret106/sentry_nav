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


git init
git add src
git status 
git commit -m "first commit"
git branch -M main
git remote add origin git@github.com:regret106/nav.git
git push -u origin main

# 1. 拉取远程最新代码并变基你的本地提交（推荐）
git pull --rebase origin main
# 2. 如果有冲突，根据提示解决冲突后继续
#    git add <冲突文件>
#    git rebase --continue
# 3. 重新推送
git push origin main
git remote set-url origin 刚才复制的密钥
