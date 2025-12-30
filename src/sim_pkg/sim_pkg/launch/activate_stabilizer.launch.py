
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    madgwick_node = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        parameters=[{
            'use_mag': False}]
    )

    stabilizer_policy = Node(
        package='sim_pkg', executable='stabilizer_policy',
        name='stabilizer_policy', output='screen'
    )

    return LaunchDescription([
        madgwick_node,
        stabilizer_policy
    ])
