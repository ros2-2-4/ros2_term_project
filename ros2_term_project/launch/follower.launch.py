from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'car_track.world'     # .sdf file
    world = os.path.join(get_package_share_directory('ros2_term_project'),
                         'worlds', world_file_name)

    pkg_gazebo_ros = get_package_share_directory('ros2_term_project')
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'car_sim.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    ros2_term_project_node = Node(
        package='ros2_term_project',
        namespace='/',
        executable='follower',
        name='follower',
        remappings=[
            ('cmd_vel', 'demo/cmd_demo'),
        ]
    )
    return LaunchDescription([
        gazebo_node,
        ros2_term_project_node,
    ])
