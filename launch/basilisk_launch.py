from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launcher = LaunchDescription()
    ws = LaunchConfiguration('ws')
    ws_arg = DeclareLaunchArgument('ws', default_value='/home/ubuntu/example_ws/')

    # Basilisk node
    basilisk_sim_node = Node(
        package="ros2basilisk",
        executable="RosBasilisk.py",
        output='screen',
        emulate_tty=True,
        remappings=[
            # ('/agent0/bsk_state_gt', '/agent0/state_nav'), # use ground truth as "state_nav"
            # ('/agent1/bsk_state_gt', '/agent1/state_nav'), # use ground truth as "state_nav"
        ],
        parameters = [
            { "workspace_dir" : ws },
            { "config_yaml" : "src/ros2basilisk/config/default_config.yaml" },
            { "dir_out" : "data/runs/bsk_default/bsk_out" }
        ],
    )
    
    launcher.add_action(ws_arg)
    launcher.add_action(basilisk_sim_node)

    return launcher
