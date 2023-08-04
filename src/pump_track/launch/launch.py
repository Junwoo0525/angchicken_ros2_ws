import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),

        Node(
            package='pump_track',
            executable='pump_pub',
            name='pump_publisher',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track',
            executable='diffbot_cont',
            name='diffbot_control',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'image_view', 'image_view', 'image:=/hist_image'],
            output='screen',
        ),
    ])
