from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    scale_str = LaunchConfiguration('scale').perform(context)
    scale = float(scale_str)
    
    scenario_str = LaunchConfiguration('scenario').perform(context)
    scenario = float(scenario_str)
    
    num_agents_str = LaunchConfiguration('num_agents').perform(context)
    num_agents = int(num_agents_str)

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('multi_agent_planner'),
        'config',
        'agent_crazyflie_config.yaml'
    )
    config_mapper = os.path.join(
        get_package_share_directory('mapping_util'),
        'config',
        'map_builder_default_config.yaml'
    )

    print("Planning with Scaled Environment: ", scale, scenario, num_agents)

    room_x = 14.0 * scale
    room_y = 6.0 * scale

    n_rob = num_agents
    
    init_line = -5.5
    
    z_pos = 2.5
    dist_start_goal = 12.0 * scale
    voxel_grid_range = [20.0, 20.0, 0.5]
    use_mapping_util = True
    free_grid = True
    save_stats = False
    
    inflated_radius = 0.046 * scale

    start_positions = []
    goal_positions = []

    # start_positions.append((x_pos, y_pos, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    # goal_positions.append((x_pos + dist_start_goal, y_pos, z_pos))
    
    if (num_agents == 8):
        rl_start = [[init_line, 1.75, 0.5],
                                    [init_line, 1.25, 0.5],
                                    [init_line, 0.75, 0.5],
                                    [init_line, 0.25, 0.5],
                                    [init_line, -0.25, 0.5],
                                    [init_line, -0.75, 0.5],
                                    [init_line, -1.25, 0.5],
                                    [init_line, -1.75, 0.5]]
    if (num_agents == 12):
        rl_start = [[init_line, 1.75, 0.5],
                                    [init_line, 1.25, 0.5],
                                    [init_line, 0.75, 0.5],
                                    [init_line, 0.25, 0.5],
                                    [init_line, -0.25, 0.5],
                                    [init_line, -0.75, 0.5],
                                    [init_line, -1.25, 0.5],
                                    [init_line, -1.75, 0.5],
                                    ######## Added new positions
                                    [init_line - 0.5, 0.25, 0.5],
                                    [init_line - 0.5, -0.25, 0.5],
                                    [init_line - 0.5, 0.75, 0.5],
                                    [init_line - 0.5, -0.75, 0.5]]
    if (num_agents == 16):
        rl_start = [[init_line, 1.75, 0.5],
                                    [init_line, 1.25, 0.5],
                                    [init_line, 0.75, 0.5],
                                    [init_line, 0.25, 0.5],
                                    [init_line, -0.25, 0.5],
                                    [init_line, -0.75, 0.5],
                                    [init_line, -1.25, 0.5],
                                    [init_line, -1.75, 0.5],
                                    ######## Added new positions
                                    [init_line - 0.5, 0.25, 0.5],
                                    [init_line - 0.5, -0.25, 0.5],
                                    [init_line - 0.5, 0.75, 0.5],
                                    [init_line - 0.5, -0.75, 0.5],
                                    [init_line - 0.5, 1.25, 0.5],
                                    [init_line - 0.5, -1.25, 0.5],
                                    [init_line - 0.5, 1.75, 0.5],
                                    [init_line - 0.5, -1.75, 0.5]]
    if (num_agents == 24):
        rl_start = [[init_line, 1.75, 0.5],
                                    [init_line, 1.25, 0.5],
                                    [init_line, 0.75, 0.5],
                                    [init_line, 0.25, 0.5],
                                    [init_line, -0.25, 0.5],
                                    [init_line, -0.75, 0.5],
                                    [init_line, -1.25, 0.5],
                                    [init_line, -1.75, 0.5],
                                    ######## Added new positions
                                    [init_line - 0.5, 0.25, 0.5],
                                    [init_line - 0.5, -0.25, 0.5],
                                    [init_line - 0.5, 0.75, 0.5],
                                    [init_line - 0.5, -0.75, 0.5],
                                    [init_line - 0.5, 1.25, 0.5],
                                    [init_line - 0.5, -1.25, 0.5],
                                    [init_line - 0.5, 1.75, 0.5],
                                    [init_line - 0.5, -1.75, 0.5],
                                    ######## Add Back Row
                                    [init_line - 1.0, 0.25, 0.5],
                                    [init_line - 1.0, -0.25, 0.5],
                                    [init_line - 1.0, 0.75, 0.5],
                                    [init_line - 1.0, -0.75, 0.5],
                                    [init_line - 1.0, 1.25, 0.5],
                                    [init_line - 1.0, -1.25, 0.5],
                                    [init_line - 1.0, 1.75, 0.5],
                                    [init_line - 1.0, -1.75, 0.5]]
        
    # Transform coords into HDSM centered
    for pos in rl_start:
        x_pos_rl = pos[0] * scale
        x_pos_trans = (x_pos_rl + (room_x / 2.0))   
        
        y_pos_rl = pos[1] * scale
        y_pos_trans = (y_pos_rl + room_y / 2.0)
        
        start_positions.append((x_pos_trans, y_pos_trans, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        goal_positions.append((x_pos_trans + dist_start_goal, y_pos_trans, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        print("[DEBUG]: Using Goals ", goal_positions)
    #0: straight 1: swap goals
    if scenario == 1:
        goal_positions.reverse()
    
    if use_mapping_util:
        for i in range(n_rob):
            params_sub = [{'id': i},
                          {'voxel_grid_range': voxel_grid_range},
                          {'free_grid': free_grid}]
            node_mapper = Node(
                package='mapping_util',
                executable='map_builder_node',
                name='map_builder_node_{}'.format(i),
                parameters=[config_mapper] + params_sub,
                # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
                # prefix=['xterm -fa default -fs 10 -hold -e'],
                output='screen',
                emulate_tty=True,
            )
#             ld.add_action(node_mapper)
            ld.add_action(node_mapper)

    for i in range(n_rob):
        params_sub = [{'state_ini': list(start_positions[i])},
                      {'n_rob': n_rob},
                      {'id': i},
                      {'goal': list(goal_positions[i])},
                      {'use_mapping_util': use_mapping_util},
                      {'drone_radius': inflated_radius},
                      {'save_stats': save_stats}]
        node = Node(
            package='multi_agent_planner',
            executable='agent_node',
            name='agent_node_{}'.format(i),
            parameters=[config] + params_sub,
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node)

    return [ld]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scale',
            default_value='5.0',
            description='environment scaling factor'
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='1.0',
            description='straight line or goal swap'
        ),
        DeclareLaunchArgument(
            'num_agents',
            default_value='8.0',
            description='number of agents to generate: [8,12,16,24]'
        ),
        OpaqueFunction(function=launch_setup)
    ])
