from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    
    node_directory = os.path.join(get_package_share_directory('edu_drive_ros2'), 'launch')
    param_config = os.path.join(node_directory,'edu_drive_faulhaber_dc.yaml')

    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)

    print(params)
    
    return LaunchDescription([
        Node(
            package='edu_drive_ros2',
            executable='edu_drive_ros2_node',
            name='edu_drive_ros2_node',
            output='screen',
            parameters=[params]
        )
    ])