import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('rm_serial_driver')
    
    # 加载参数文件
    params_file = os.path.join(pkg_path, 'config', 'serial_driver.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)
    
    # 创建serial_driver节点
    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[params],
        ros_arguments=['--ros-args', ],
    )
    
    return LaunchDescription([
        serial_driver_node,
    ]) 