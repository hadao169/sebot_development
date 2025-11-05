#This launch file starts the robot_state_publisher node to publish the robot's state to TFs based on the URDF description.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    colcon_prefix_path = os.getenv('COLCON_PREFIX_PATH').split("/install")[0]
    urdf_file_name = 'my_robot.urdf'
    urdf = os.path.join(
        colcon_prefix_path,
        'config',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    print("Robot description loaded from: ", urdf)


    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),

        Node(
            package='motordriver',
            executable='motordriver',
            name='motordriver_node',
            output='screen',
            parameters=[os.path.join(
              colcon_prefix_path,
              'config',
              'params.yaml')]
        ),

        Node(
            package='diffdrive',
            executable='odom',
            name='odom_node',
            output='screen',
            parameters=[os.path.join(
              colcon_prefix_path,
              'config',
              'params.yaml')]
        ),

        Node(
            package='diffdrive',
            executable='cmd_vel',
            name='cmd_vel_node',
            output='screen',
            parameters=[os.path.join(
              colcon_prefix_path,
              'config',
              'params.yaml')]
        ),
        Node(
            package='obstacle_avoid',
            executable='obstacle_avoid_node',
            name='obstacle_avoid_node',
            output='screen',
            parameters=[os.path.join(
              colcon_prefix_path,
              'config',
              'params.yaml')]
        ),
    ])

