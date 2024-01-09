from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf_tutorial_path = get_package_share_directory('fl_robot_py')
    print('urdf_tutorial_path', urdf_tutorial_path)
    default_model_path = urdf_tutorial_path + '/urdf/fl_robot01.xacro'
    print('default_model_path', default_model_path)
    
    default_rviz_config_path = urdf_tutorial_path + '/rviz/defult.rviz'
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')
    
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig',         default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration('model')]),
                                       value_type=str)
                                  
    world_file = DeclareLaunchArgument(name = 'House', default_value = "$(find fl_robot_py)/worlds/closed_room_short.world")
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
             )
#    gazebo = ExecuteProcess(
#            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
#            output='screen'),
                                  

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gazebo_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", "macroed"],
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        gazebo
    ])
