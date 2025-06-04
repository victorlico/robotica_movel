from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    turtlebot3_gazebo_path = os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'), 
                    'worlds', 
                    'turtlebot3_world.world'
    )

    rviz_config = os.path.join(
                    get_package_share_directory('andino_navigation'), 
                    'rviz', 
                    'nav2_default_view.rviz'
    )    
    
    andino_gz_launch_path = os.path.join(
        get_package_share_directory('andino_gz_classic'),
        'launch',
        'andino_one_robot.launch.py'
    )

    andino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(andino_gz_launch_path),
        launch_arguments={'rviz': 'true',
                            'use_sim_time': 'true',
                            'world': turtlebot3_gazebo_path,
                            'initial_pose_x': '-2.00',
                            'initial_pose_y': '-0.5',
                            'initial_pose_z': '0.01',
                            'initial_pose_yaw': '0.00',
                            'rviz_config_file': rviz_config}.items()
    )

    remaps = Node(
        package='topic_tools', 
        executable='relay',
        arguments=['/camera/image_raw', '/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        andino_launch,
        remaps
    ])
