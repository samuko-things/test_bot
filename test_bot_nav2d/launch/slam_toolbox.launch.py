import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro



def generate_launch_description():

    nav_pkg_path = get_package_share_directory('test_bot_nav2d')
    description_pkg_path = get_package_share_directory('test_bot_description')
    
    
    world_file_name = 'test_world.world'
    world_path = os.path.join(description_pkg_path, 'world', world_file_name)
    
    rviz_config_file = os.path.join(description_pkg_path,'config','robot_sim_view.rviz')

    ekf_file_path = os.path.join(description_pkg_path,'config', 'ekf.yaml')
    
    
    
    
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(description_pkg_path,'launch','rsp.launch.py')]
            ), 
            launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )
    
    
    
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )




    gazebo_params_file = os.path.join(description_pkg_path,'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
                ),
                launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )



    # entity_name = 'mobo_bot'
    # initial spawn position
    x_pos = 0; y_pos = 0; z_pos = 0.033
    #initial spawn orientation
    roll = 0; pitch = 0; yaw = 0
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'test_bot',
            # '-x', str(x_pos), '-y', str(y_pos), '-z', str(z_pos),
            # '-R', str(roll), '-P', str(pitch), '-Y', str(yaw)
            ],
        output='screen')



    # Start robot localization using an Extended Kalman filter
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file_path, {'use_sim_time': 'true'}])



    slam_mapping_param_file = os.path.join(nav_pkg_path,'config','my_mapper_params_online_async.yaml')
    slam_mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_mapping_param_file],
    #     remappings=[
    #     # ('/odom', '/turtlesim1/turtle1/pose'),
    #     # ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    # ]
    )
    
    
    

     # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='SDF world file',
        ),
        
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        slam_mapping_node
        # node_ekf
    ])
    


   
