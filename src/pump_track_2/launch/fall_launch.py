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
            package='pump_track_2',
            executable='usb_camera_node',
            name='usb_camera_node',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track_2',
            executable='marker_detection',
            name='marker_detection',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),


    ])
