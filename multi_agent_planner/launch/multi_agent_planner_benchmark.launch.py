from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # get config file
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
    
    stretch_factor = 5.0

    # define params
    room_x = 14.0 * stretch_factor
    room_y = 6.0 * stretch_factor

    n_rob = 8
    dist_between_rob = 0.5 * stretch_factor
    
    x_pos_rl = -5.5 * stretch_factor# 0.0
    x_pos = (x_pos_rl + room_x / 2.0)
    
    y_pos_rl = -1.75 * stretch_factor
    y_pos = (y_pos_rl + room_y / 2.0)
    
    z_pos = 2.5
    dist_start_goal = 12.0 * stretch_factor
    voxel_grid_range = [20.0, 20.0, 0.5]
    use_mapping_util = True
    free_grid = True
    save_stats = False
    # calculate equidistant start and goal positions on the same line
    start_positions = []
    goal_positions = []
    # start_positions.append((x_pos, -1.75 + 3.0, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    start_positions.append((x_pos, y_pos, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    goal_positions.append((x_pos + dist_start_goal, y_pos, z_pos))
    for i in range(n_rob-1):
        y = start_positions[0][1] + (i+1)*dist_between_rob
        start_positions.append((x_pos,  y, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        goal_positions.append((x_pos + dist_start_goal, y, z_pos))
    # start_positions.append((x_pos, 0.5, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    # goal_positions.append((x_pos + dist_start_goal, 5.0, z_pos))
    # for i in range(n_rob-1):
    #     y = start_positions[0][1] + (i+1)*dist_between_rob
    #     start_positions.append((x_pos,  y, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    #     goal_positions.append((x_pos + dist_start_goal, y, z_pos))
    
    # with open("src/multi_agent_pkgs/multi_agent_planner/launch/run_6_goals", "rb") as fp:   # Unpickling
    #     goal_positions = pickle.load(fp)
        
    # with open("src/multi_agent_pkgs/multi_agent_planner/launch/run_6_start", "rb") as fp:   # Unpickling
    #     start_positions = pickle.load(fp)
        
    print(start_positions)

    # create mapping nodes
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
            ld.add_action(node_mapper)
    # create nodes
    for i in range(n_rob):
        params_sub = [{'state_ini': list(start_positions[i])},
                      {'n_rob': n_rob},
                      {'id': i},
                      {'goal': list(goal_positions[i])},
                      {'use_mapping_util': use_mapping_util},
                      {'save_stats': save_stats}]
        node = Node(
            package='multi_agent_planner',
            executable='agent_node',
            name='agent_node_{}'.format(i),
            parameters=[config] + params_sub,
            # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
            # prefix=['xterm -fa default -fs 10 -hold -e'],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node)

    return ld