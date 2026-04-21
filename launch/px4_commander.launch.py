from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    params_file = os.path.join(
        get_package_share_directory(f'px4_commander'),
        'config',
        'params.yaml',
    )

    px4_commander_server = Node(
        package='px4_commander',
        executable='px4_commander_server',
        name='px4_commander_server',
        namespace='px4_commander',
        output='screen',
        parameters=[{'config_file': params_file}],
    )

    ld.add_action(px4_commander_server)

    return ld