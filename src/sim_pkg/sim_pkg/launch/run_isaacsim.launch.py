# sim_pkg/launch/run_isaacsim.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    script = DeclareLaunchArgument(
        "script",
        default_value="/home/jliu/isaac_ws/src/sim_pkg/sim_pkg/scripts/run_isaacsim.py",
        description="Absolute path to the Python script to run",
    )

    python_bin = DeclareLaunchArgument(
        "python_bin",
        default_value="python3",
        description="Python interpreter to use (e.g., Isaac Sim's python.sh)",
    )

    # Launch the IsaacSim Python script
    isaac_proc = ExecuteProcess(
        cmd=[LaunchConfiguration("python_bin"), LaunchConfiguration("script")],
        output="screen",
        shell=False,
    )

    # Launch your ROS 2 node
    box_node = Node(
        package="sim_pkg",
        executable="box_cmd_node",
        name="box_cmd_node",
        output="screen",
    )

    return LaunchDescription([
        script,
        python_bin,
        isaac_proc,
        box_node,
    ])
