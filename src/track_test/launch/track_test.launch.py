import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'angchicken'
    package_path = os.path.join(get_package_share_directory(package_name))
    
    robot_file = 'testcar.urdf'
    urdf_file = os.path.join(package_path, 'urdf', robot_file)

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description':doc.toxml()}

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file_name = 'track_sample_testcar.world'
    world = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)

    return LaunchDescription([
        # gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world':world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        ),
    ])