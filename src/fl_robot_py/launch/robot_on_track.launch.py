from ament_index_python.packages import get_package_share_path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('fl_robot_py')
    print('urdf_tutorial_path ==== ', urdf_tutorial_path)
    default_model_path = urdf_tutorial_path / 'urdf/fl_robot01.xacro'
    print('default_model_path ==== ', default_model_path)
    default_rviz_config_path = urdf_tutorial_path / 'rviz/urdf.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    package_name = 'fl_robot_py'
    world = LaunchConfiguration('world')
    world_file_path = 'worlds/bigHouse.world'
#    world_file_path = 'worlds/closed_room_short.world'
    robot_name_in_model = 'mv_robot1'
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    world_path = os.path.join(pkg_share, world_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')

    # Pose where we want to spawn the robot
    spawn_x_val = '1.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '-1.57'

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
      name='gui',
      default_value='true',
      description='Flag to enable joint_state_publisher_gui')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true')

#    declare_urdf_model_path_cmd = DeclareLaunchArgument(
#      name='urdf_model', 
#      default_value=default_model_path, 
#      description='Absolute path to robot urdf file')


    declare_simulator_cmd = DeclareLaunchArgument(
      name='headless',
      default_value='False',
      description='Whether to execute gzclient')
    declare_use_simulator_cmd = DeclareLaunchArgument(
      name='use_simulator',
      default_value='True',
      description='Whether to start the simulator')
    declare_world_cmd = DeclareLaunchArgument(
      name='world',
      default_value=world_path,
      description='Full path to the world model file to load')
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
      condition=IfCondition(use_simulator),
      launch_arguments={'world': world}.items())

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
      condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))




    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Launch the robot
    spawn_entity_cmd = Node(
      package='gazebo_ros', 
      executable='spawn_entity.py',
      arguments=['-entity', robot_name_in_model, 
                  '-topic', 'robot_description',
                      '-x', spawn_x_val,
                      '-y', spawn_y_val,
                      '-z', spawn_z_val,
                      '-Y', spawn_yaw_val],
                      output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    drive_node = Node(
        package='fl_robot_py',
        executable='drive_it',
        name='drive_fl_robot',
        output='screen'
    )

    return LaunchDescription([
        declare_use_joint_state_publisher_cmd,
#        declare_urdf_model_path_cmd,
        gui_arg,
        model_arg,
        rviz_arg,
        declare_simulator_cmd,
        declare_use_sim_time_cmd,
        declare_use_simulator_cmd,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        spawn_entity_cmd,
        robot_state_publisher_node,
        rviz_node,
        declare_world_cmd,
        start_gazebo_server_cmd, 
        start_gazebo_client_cmd,
        drive_node,
    ])
