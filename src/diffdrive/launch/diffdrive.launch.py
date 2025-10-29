#This launch file starts the robot_state_publisher node to publish the robot's state to TFs based on the URDF description.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    #NAMESPACE = 'SeBotxx' # replace xx with your SeBot's identifier, e.g., the last byte of the IP address.
    #FRAME_PREFIX = NAMESPACE+"_"
    # Working directory
    colcon_prefix_path = os.getenv('COLCON_PREFIX_PATH').split("/install")[0] #determine the root of directory tree
    # Read URDF file into robot_desc variable
    urdf_file_name = 'robot.urdf'
    urdf = os.path.join( #construct absolute path to urdf file
        colcon_prefix_path,
        'config',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    print("Robot description loaded from: ", urdf)

    return LaunchDescription([

        #Define command-line arguments named 'use_sim_time' => use real time or simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Node to launch
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            #parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc, 'frame_prefix': FRAME_PREFIX}],# uncomment this line if you want to use a multi-robot namespace arrangement. Comment out the next line accordingly.
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            #arguments=[urdf], # 26.5.2025 This line is redundant, robot_state_publisher does not parse command line arguments separately. The URDF file is passed to it as parameters.
            #namespace=NAMESPACE, # uncomment this line if you want to use a multi-robot namespace arrangement.
        ),

        ## Add conversion map->[namespace]_odom so that all robots are included in the same TF tree.
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    name='map_to_robot_odom',
        #    namespace=NAMESPACE,
        #    arguments=['0', '0', '0', '0', '0', '0',  # x y z yaw pitch roll
        #               'map', f'{FRAME_PREFIX}odom'],
        #    output='screen'
        #),

        # 4. NODE MỚI: RViz 2
        # Node này đọc TF và hiển thị mô hình 3D.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            output='screen',
            # arguments=['-d', rviz_config_file], # Mở chú thích nếu sử dụng file cấu hình RViz riêng
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='motordriver',
            executable='motordriver',
            name='motordriver_node',
            #namespace = NAMESPACE,
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
            #namespace = NAMESPACE,
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
            #namespace = NAMESPACE,
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
            #namespace = NAMESPACE,
            output='screen',
            parameters=[os.path.join(
              colcon_prefix_path,
              'config',
              'params.yaml')]
        ),

        # Node(
        #     package='battery',
        #     executable='battery_republish',
        #     name='battery_republish_node',
        #     parameters=[os.path.join(
        #       colcon_prefix_path,
        #       'config',
        #       'params.yaml')],
        #     output='screen',
        #   ),

        # Node(
        #     package='battery',
        #     executable='battery_alert',
        #     name='battery_alert_node',
        #     parameters=[os.path.join(
        #       colcon_prefix_path,
        #       'config',
        #       'params.yaml')],
        #     output='screen',
        # ),

        ## Add conversion map->[namespace]_odom so that all robots are included in the same TF tree.
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    name='map_to_robot_odom',
        #    namespace=NAMESPACE,
        #    arguments=['0', '0', '0', '0', '0', '0',  # x y z yaw pitch roll
        #               'map', f'{FRAME_PREFIX}odom'],
        #    output='screen'
        #),
    ])

