from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


######################################################################
# This is ROS launch script
# Instruction:
# - Follow README.md
# - Run : ros2 launch ros2basilisk lqr_launch.py ws:="/home/ubuntu/example_ws/" 
#   where 'ws' variable is set to appropriate workspace
######################################################################

run_name = "default_run"

def generate_launch_description():

    launcher   = LaunchDescription()

    # User can optionally specify 'ws' and 'config' in the argument of ros2 launch
    ws         = LaunchConfiguration('ws')
    config     = LaunchConfiguration('config')
    ws_arg     = DeclareLaunchArgument('ws', default_value="/home/ubuntu/example_ws/")
    config_arg = DeclareLaunchArgument('config', default_value="src/ros2basilisk/config/default_config.yaml")

    agent_names = ["agent0", "agent1"]
    
    # Navigation node for each agent
    for name in agent_names:
        simple_nav = Node(
            package="ros2basilisk",
            executable="SimpleNav.py",
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/sc_name/bsk_state_gt', '/' + name + '/bsk_state_gt'),
                ('/sc_name/state_nav', '/' + name + '/state_nav'),
                ], 
            parameters = [
                { "workspace_dir" : ws },
                { "config_yaml" : config },
            ],
        )
        launcher.add_action(simple_nav)

    # LQR controller node
    lqr_node = Node(
        package="ros2basilisk",
        executable="FormationControl.py",
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/sc_self/state_nav', '/agent1/state_nav'),
            ('/sc_ref/state_nav', '/agent0/state_nav'),
            # ('/sc_self/mrp_des_sensor', '/agent1/mrp_des_sensor'), # sensor pointing guidance cmd
            ('/sc_self/thr_cmd', '/agent1/thr_cmd'),
            ('/sc_self/mrp_des', '/agent1/mrp_des'),
        ], 
        parameters = [
            { "workspace_dir" : ws },
            { "config_yaml" : config },
            { 'enable_att_passthru' : False },
        ],
    )

    
    # Attitude Controller node
    mrp_ctrl = Node(
        package="ros2basilisk",
        executable="AttitudeCtrlAllocNode.py",
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/sc_name/mrp_des', '/agent1/mrp_des'),
            ('/sc_name/state_nav', '/agent1/state_nav'),
            ('/sc_name/rw_torque_cmd', '/agent1/rw_torque_cmd'),
        ], 
        parameters = [
            { "sc_name" : 'agent1' },
            { "workspace_dir" : ws },
            { "config_yaml" : config },
            { "Kp" : 5.0 },
            { "Kd" : 10.0 },
            { "save_debug" : False },
            { "output_dir" : "data/runs/" + run_name + "/mrp_ctrl_alloc/"  },
        ],
    )

    # Basilisk node
    basilisk_sim_node = Node(
        package="ros2basilisk",
        executable="RosBasilisk.py",
        output='screen',
        emulate_tty=True,
        parameters = [
            { "workspace_dir" : ws },
            { "config_yaml" : config },
            { "dir_out" : "data/runs/" + run_name + "/bsk_data_out/" },
            { "delayStep" : 0.02 }, 
            { "timeStep" : 0.2 }, 
            { "simTime" : 3600.0 } ],
    )

    launcher.add_action(mrp_ctrl)
    launcher.add_action(lqr_node)
    launcher.add_action(ws_arg)
    launcher.add_action(config_arg)

    timer_action = TimerAction(period = 2.0, actions = [basilisk_sim_node])
    launcher.add_action(timer_action)

    return launcher
