import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    leader = LaunchConfiguration('leader').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    n_drones = int(LaunchConfiguration('n_drones').perform(context))
    log_dir = LaunchConfiguration('log_dir').perform(context)
    enable_delta = LaunchConfiguration('enable_delta').perform(context) == 'true'
    enable_delta_dot = LaunchConfiguration('enable_delta_dot').perform(context) == 'true'
    path_offset = LaunchConfiguration('path_offset').perform(context)
    path_offset_north = LaunchConfiguration('path_offset_north').perform(context)
    log_tag = LaunchConfiguration('log_tag').perform(context)
    leader_speed = LaunchConfiguration('leader_speed').perform(context)
    leader_amplitude = LaunchConfiguration('leader_amplitude').perform(context)
    leader_wavelength = LaunchConfiguration('leader_wavelength').perform(context)
    formation_config = os.path.expanduser(
        LaunchConfiguration('formation_config').perform(context))

    # Every node reads leader start pose, spawns and slots from the same file.
    common_params = {
        'n_drones': n_drones,
        'log_dir': log_dir,
        'formation_config': formation_config,
    }

    # One leader executable; --leader picks its path.
    leader_params = dict(common_params)
    leader_params['path'] = leader
    if leader_speed:
        leader_params['speed'] = float(leader_speed)
    if leader_amplitude:
        leader_params['amplitude'] = float(leader_amplitude)
    if leader_wavelength:
        leader_params['wavelength'] = float(leader_wavelength)
    if path_offset:
        leader_params['path_offset'] = float(path_offset)
    if path_offset_north:
        leader_params['path_offset_north'] = float(path_offset_north)

    follower_params = dict(common_params)
    follower_params['enable_delta'] = enable_delta
    follower_params['enable_delta_dot'] = enable_delta_dot

    # formation_logger uses output_dir + num_followers, not
    # log_dir/n_drones -- match the parameter names they actually declare.
    logger_params = {
        'num_followers': n_drones,
        'tag': log_tag,
        'formation_config': formation_config,
    }
    # Empty log_dir -> keep the logger's own default (~/sim_logs/sitl/standalone).
    if log_dir:
        logger_params['output_dir'] = log_dir
    actions = [
        Node(package='swarm_formation', executable='path_leader',
             name='leader', output='screen', parameters=[leader_params]),

        Node(package='swarm_formation', executable='follower_controller',
             name='follower_controller', output='screen', parameters=[follower_params]),

        Node(package='swarm_formation', executable='swarm_visualizer',
             name='swarm_visualizer', output='screen', parameters=[common_params]),

        Node(package='swarm_formation', executable='formation_logger',
             name='formation_logger', output='screen', parameters=[logger_params]),
    ]

    if rviz == 'true':
        actions.append(ExecuteProcess(cmd=['rviz2'], output='screen'))

    return actions


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'leader', default_value='sinusoid',
            choices=['sinusoid', 'figure8', 'circle', 'straight_line'],
            description='Virtual leader reference path generator.'),

        DeclareLaunchArgument(
            'rviz', default_value='true',
            choices=['true', 'false'],
            description='Launch rviz2 alongside the stack.'),

        DeclareLaunchArgument(
            'n_drones', default_value='4',
            description='Number of follower UAVs (1-4). Must match the '
                        'instance count started by start_swarm.sh.'),

        DeclareLaunchArgument(
            'log_dir', default_value='',
            description='Directory for the formation CSV. Empty = node default.'),

        DeclareLaunchArgument(
            'enable_delta', default_value='true',
            choices=['true', 'false'],
            description="Follower's carousel heading correction (Delta)."),

        DeclareLaunchArgument(
            'enable_delta_dot', default_value='true',
            choices=['true', 'false'],
            description="Follower's dDelta/dt feedforward (false = pointwise)."),

        DeclareLaunchArgument(
            'path_offset', default_value='',
            description='East shift of the path [m] = initial cross-track '
                        'error; the leader flies onto the path (VF mode). '
                        'Empty or 0 = exact mode. Any path.'),

        DeclareLaunchArgument(
            'path_offset_north', default_value='',
            description='North shift of the path [m] (with path_offset = East). '
                        'Empty or 0 = no North shift.'),

        DeclareLaunchArgument(
            'log_tag', default_value='',
            description='Suffix appended to the logger CSV filename.'),

        DeclareLaunchArgument(
            'leader_speed', default_value='',
            description='Leader ground speed. Empty = node default.'),

        DeclareLaunchArgument(
            'leader_amplitude', default_value='',
            description='Sinusoid amplitude A. Empty = node default.'),

        DeclareLaunchArgument(
            'leader_wavelength', default_value='',
            description='Sinusoid wavelength. Empty = node default.'),

        DeclareLaunchArgument(
            'formation_config',
            default_value=os.path.join(get_package_share_directory('swarm_formation'),
                                       'config', 'formation.yaml'),
            description='Leader start pose, follower spawns and formation slots. '
                        'start_swarm.sh must spawn Gazebo from the same file.'),

        OpaqueFunction(function=launch_setup),
    ])
