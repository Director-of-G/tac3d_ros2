import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Specify the path to the MuJoCo model file
    config_url = os.path.join(
            get_package_share_directory('tac3d_ros2'),
            'config',
            'tac3d_leap_config.yaml'
        )
    config = yaml.safe_load(open(config_url, 'r'))

    tac3d_client_list = []
    for _, tac3d_config in config['tac3d'].items():
        index = len(tac3d_client_list)
        node = Node(
            package='tac3d_ros2',
            executable='single_tac3d_node.py',
            name=f'single_tac3d_reader_node_{index}',
            output='screen',
            parameters=[tac3d_config, {'index': index}]
        )
        tac3d_client_list.append(node)

    return LaunchDescription(
        tac3d_client_list + [
            Node(
                package='tac3d_ros2',
                executable='contact_state_publisher.py',
                name='contact_state_publisher_node',
                output='screen',
                parameters=[{'config_url': config_url}]
            )
    ])

if __name__ == '__main__':
    generate_launch_description()
