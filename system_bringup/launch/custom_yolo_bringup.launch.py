from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'launch',
                'robot.launch.py'
            )
        )
    )

    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        parameters=[
            {'format': 'RGB888'},
            {'width': 640},
            {'height': 640}
        ]
    )

    people_mapper = Node(
        package='people_mapper_pkg',
        executable='people_mapper_node'
    )

    ble_warn = Node(
        package='ble_warn',
        executable='ble_warn'
    )

    return LaunchDescription([
        turtlebot_launch,
        camera_node,
        people_mapper,
        ble_warn
    ])
