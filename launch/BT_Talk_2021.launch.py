import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    behavior_directory = get_package_share_directory('bt_talk_2021')
    xml_path = os.path.join(behavior_directory, 'config', 'bt_fall_2021.xml')
    ld = LaunchDescription()
    behavior_node = Node(package='bt_talk_2021',
                         executable='BT_Talk_2021',
                         name='BT_Talk_2021_node',
                         parameters=[{
                             'xml_path': xml_path
                         }],
                         prefix=['xterm -e gdb --args'],
                         respawn=False,
                         output='screen')
    ld.add_action(behavior_node)

    return ld
