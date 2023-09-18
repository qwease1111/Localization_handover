from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_uwb_localization',
            executable='ros2_uwb_node',
            name='ros2_uwb_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("ros2_uwb_localization"), 'params', 'uwb.yaml')],
        ),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='b_uwb_localization', 
            output='screen',                       
            arguments=['0', '0', '0', '0', '0', '1', 'base_link', 'uwb_link']            		
        ),
])
