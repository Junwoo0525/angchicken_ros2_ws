from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  test_code = Node(
    package='lidar_pkg',
    executable='lidar_code',
    output='screen',
    remappings=[
        ('laser_scan', '/scan')
    ]
  )

  return LaunchDescription([
    lidar_code
  ])
