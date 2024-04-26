"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'final.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    Exec_path = os.path.join(get_package_share_directory('tello_gazebo'))
    Exec_path = os.path.abspath(os.path.join(Exec_path, "../../../..","src/","drone_racing_ros2/","tello_ros/","tello_gazebo/","src/drone_controller.py"))
    
    print(Exec_path)
    return LaunchDescription([
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Spawn tello.urdf
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '-2', '1', '1.57079632679']),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen',
             namespace=ns),

        # Joystick controller, generates /namespace/cmd_vel messages
        Node(package='tello_driver', executable='tello_joy_main', output='screen',
             namespace=ns),

        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'python3',
            Exec_path
        ], output='screen'),

    ])
