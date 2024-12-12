import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    tac3d_server_root = config['server_url']
    terminal_commands = ["gnome-terminal", "--window"]
    for i, (_, finger_config) in enumerate(config['tac3d'].items()):
        serial = finger_config['serial']
        cam_id = finger_config['cam_id']
        port = finger_config['port']
        command = f"cd {tac3d_server_root} && ./Tac3D -c ./config/{serial} -d {cam_id} -i 127.0.0.1 -p {port}; exec bash"
        if i > 0:
            terminal_commands.extend(["--tab", "-e", f"bash -c '{command}'"])
        else:
            terminal_commands.extend(["-e", f"bash -c '{command}'"])
    print(terminal_commands)
    tac3d_server_list = [
        ExecuteProcess(
            cmd=terminal_commands,
            output="screen"
        )
    ]

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
        tac3d_server_list + \
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
