# from launch import LaunchDescription
# from launch_ros.actions import Node
# import os
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():

#     ld = LaunchDescription()
#     # Create the NatNet client node
#     config = os.path.join(
#         get_package_share_directory('env_builder'),
#         'config',
#         # 'env_default_config.yaml'
#         # 'env_long_config.yaml'
#         # 'env_loop_config.yaml'
#         # 'env_new_config.yaml'
#         # 'env_crazyflie_config.yaml'
#         'env_rl_config.yaml'
#     )
#     params_sub = [{'publish_period': 0.1}]
#     env_builder_node = Node(
#         package='env_builder',
#         executable='env_builder_node',
#         name='env_builder_node',
#         parameters=[config] + params_sub,
#         # prefix=['xterm -fa default -fs 10 -e gdb -ex run --args'],
#         output='screen',
#         emulate_tty=True
#     )

#     ld.add_action(env_builder_node)
#     return ld

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('env_builder'),
        'config',
        'env_rl_config.yaml'
    )

    # pass `config_file:=...`
    declare_cfg = DeclareLaunchArgument(
        'config_file',
        default_value=default_cfg,
        description='path to scenario YAML'
    )
    cfg = LaunchConfiguration('config_file')

    env_builder = Node(
        package='env_builder',
        executable='env_builder_node',
        name='env_builder_node',
        parameters=[cfg, {'publish_period': 0.1}],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        declare_cfg,
        env_builder,
    ])
