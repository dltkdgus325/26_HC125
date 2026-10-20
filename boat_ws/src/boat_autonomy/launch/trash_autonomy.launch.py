"""Launch LOS, trash detection, arbitration, and diagnostics."""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_nodes(context):
    """Create nodes using the route selected in the LOS parameter YAML."""
    params_file = LaunchConfiguration('params_file').perform(context)
    los_params_file = LaunchConfiguration('los_params_file').perform(context)
    logging_params_file = LaunchConfiguration(
        'logging_params_file'
    ).perform(context)
    with Path(los_params_file).open('r', encoding='utf-8') as stream:
        los_config = yaml.safe_load(stream)
    try:
        parameters = los_config['los_controller']['ros__parameters']
        path_file = parameters['path_file']
    except (KeyError, TypeError) as exc:
        raise ValueError(
            'los_params_file must define '
            'los_controller.ros__parameters.path_file'
        ) from exc

    return [
        Node(
            package='boat_autonomy', executable='boat_comm_node',
            name='boat_uart_node', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='boat_autonomy', executable='camera_publisher',
            name='camera_publisher_node', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='boat_autonomy', executable='camera_recorder',
            name='camera_recorder', output='screen', prefix='nice -n 10',
            parameters=[logging_params_file],
        ),
        Node(
            package='boat_autonomy', executable='mission_recorder',
            name='mission_recorder', output='screen', prefix='nice -n 10',
            parameters=[logging_params_file],
        ),
        Node(
            package='boat_autonomy', executable='los_controller',
            name='los_controller', output='screen',
            parameters=[
                los_params_file,
                # Integrated mode: trash_controller alone owns /boat/cmd.
                {'command_topic': '/autonomy/global_cmd'},
            ],
        ),
        Node(
            package='boat_autonomy', executable='trash_detection',
            name='trash_detection_node', output='screen',
            parameters=[params_file],
            prefix='/home/rodin/boat_venv/bin/python3',
        ),
        Node(
            package='boat_autonomy', executable='trash_controller',
            name='trash_controller', output='screen',
            parameters=[params_file],
        ),
        Node(
            package='boat_autonomy', executable='autonomy_dashboard',
            name='autonomy_dashboard', output='screen',
            parameters=[params_file, {'path_file': str(path_file)}],
        ),
    ]


def generate_launch_description():
    """Build a launch description with one shared route source."""
    package_share = FindPackageShare('boat_autonomy')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'trash_detection_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'los_params_file',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'los_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'logging_params_file',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'data_logging_params.yaml',
            ]),
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
