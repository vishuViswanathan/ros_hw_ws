import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = "fl_robot01.xacro"
    urdf = os.path.join(
        get_package_share_directory('fl_robot_py'), 'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
             )

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'demo', '-database', 'double_pendulum_with_base'],
                        output='screen')
    return LaunchDescription([#        demo_nodes,
        gazebo,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
#        Node(
#            package='gazebo_ros',
#            executable='spawn_entity.py',
#            name='spawn_model',
#            arguments=[urdf],
#            output='screen',
#        ),
         Node(
            package='controller_manager',
            executable='spawner',
            name='fl_robot01_controller_spawner',
            arguments=['joint_state_controller',
              'right_wheel_controller',
              'left_wheel_controller',
              'camera_controller']
            
        ),
 
    ])

