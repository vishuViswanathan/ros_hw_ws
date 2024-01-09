from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    
    package_name='fl_robot_py' #<--- CHANGE ME

    rsp_no_pub = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_no_pub.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )


    gazebo_models_path = 'models'

    urdf_tutorial_path = get_package_share_path('fl_robot_py')
    # print('urdf_tutorial_path ==== ', urdf_tutorial_path)
    default_model_path = urdf_tutorial_path / 'urdf/robot.urdf.xacro'
    print('type(default_model_path) ==== ', type(default_model_path))
    print('type(str(default_model_path)) ==== ', type(str(default_model_path)))
    
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
#    world_file_path = 'worlds/warehouse.world'  # 'worlds/neighborhood.world'  #'worlds/inventory.world' # or bigHouse.world
    world_file_path = 'worlds/neighborhood.world'  #'worlds/inventory.world' # or bigHouse.world
    robot_name_in_model = 'mv_robot1'
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    world_path = os.path.join(pkg_share, world_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    use_composition =LaunchConfiguration('use_composition')
    static_map_path = os.path.join(pkg_share, 'maps', 'warehouse_world/002/map.yaml')
#    static_map_path = os.path.join(pkg_share, 'maps', 'my_map20230330.yaml')
    map_yaml_file = LaunchConfiguration('map') 
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    autostart = LaunchConfiguration('autostart')

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Pose where we want to spawn the robot
    spawn_x_val = '1.0'
    spawn_y_val = '0.5'
    spawn_z_val = '0.2'
    spawn_yaw_val = '-1.57'

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    declare_namespace_cmd = DeclareLaunchArgument(
      name='namespace',
      default_value='',
      description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
      name='use_namespace',
      default_value='False',
      description='Whether to apply a namespace to the navigation stack')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
      name='gui',
      default_value='true',
      description='Flag to enable joint_state_publisher_gui')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time',
      default_value='True',
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


    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
      name='use_robot_state_pub',
      default_value='True',
      description='Whether to start the robot state publisher')



    start_robot_state_publisher_cmd = Node(
      condition=IfCondition(use_robot_state_pub),
      package='robot_state_publisher',
      executable='robot_state_publisher',
      namespace=namespace,
      parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
      remappings=remappings,
      arguments=[str(default_model_path)])

    #robot_state_publisher_node = Node(
    #    package='robot_state_publisher',
    #    executable='robot_state_publisher',
    #    parameters=[{'robot_description': robot_description}]
    #)



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
    
      # Launch the ROS 2 Navigation Stack
    declare_slam_cmd = DeclareLaunchArgument(
      name='slam',
      default_value='False',
      description='Whether to run SLAM')
    
    declare_slam_cmd = DeclareLaunchArgument(
      name='use_composition',
      default_value='False',
      description='Whether to run use_composition')

    declare_params_file_cmd = DeclareLaunchArgument(
      name='params_file',
      default_value=nav2_params_path,
      description='Full path to the ROS2 parameters file to use for all launched nodes')
      
    declare_bt_xml_cmd = DeclareLaunchArgument(
      name='default_bt_xml_filename',
      default_value=behavior_tree_xml_path,
      description='Full path to the behavior tree xml file to use')
        
    declare_autostart_cmd = DeclareLaunchArgument(
      name='autostart', 
      default_value='true',
      description='Automatically startup the nav2 stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
      name='map',
      default_value=static_map_path,
      description='Full path to map file to load')
        
    start_ros2_navigation_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
      launch_arguments = {'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'use_composition': use_composition,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

#$$$$$$$$$$$$$$$$$$ for hardware START
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


#$$$$$$$$$$$$$$$$$$ for hardware END

    return LaunchDescription([

        declare_use_joint_state_publisher_cmd,
#        declare_urdf_model_path_cmd,
        gui_arg,
        model_arg,
        rviz_arg,
        declare_simulator_cmd,
        declare_use_sim_time_cmd,
        declare_use_simulator_cmd,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_slam_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        declare_autostart_cmd,
        declare_map_yaml_cmd,
        declare_world_cmd,
        declare_use_robot_state_pub_cmd,
        
        #joint_state_publisher_node,
        #joint_state_publisher_gui_node,
#        rsp_no_pub,
        spawn_entity_cmd,

#        delayed_controller_manager,
#        delayed_diff_drive_spawner,
#        delayed_joint_broad_spawner,


#        robot_state_publisher_node,
        start_robot_state_publisher_cmd,
        rviz_node,
        start_gazebo_server_cmd, 
#        start_gazebo_client_cmd,
        start_ros2_navigation_cmd        
    ])

