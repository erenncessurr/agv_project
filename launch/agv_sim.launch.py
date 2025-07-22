import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node




def generate_launch_description():

    package_name='agv_project' 

    agv = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','agv.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/tricycle_controller/cmd_vel')]
        )
    
    """
    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            parameters=[{'use_sim_time': 'true'}],
            remappings=[('/cmd_vel_in','/tricycle_controller/cmd_vel'),
                        ('/cmd_vel_out','/tricycle_controller/cmd_vel')]
          )
          """

    teleop_keyboard = Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            prefix="xterm -e",
            parameters=[{'stamped': False}],
            remappings=[('cmd_vel','/tricycle_controller/cmd_vel')])
    

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'agv',
                                   '-z', '0.1'],
                        output='screen')

    #robot_description = {'robot_description': agv}

    robot_controllers = PathJoinSubstitution(
        [
        FindPackageShare(package_name),
        'config',
        'controllers_tricycle.yaml',
        ]
    )

    tricycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'tricycle_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
            ],
    )
    
    delayed_spawn_entity = TimerAction(
        period=1.0,  # saniye olarak gecikme
        actions=[spawn_entity]
        )
    
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_params.yaml')

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Launch them all!
    return LaunchDescription([
        agv,
        world_arg,
        twist_mux,
        gazebo,
        tricycle_controller_spawner,
        joint_state_broadcaster_spawner,
        delayed_spawn_entity,
        ros_gz_bridge,
        teleop_keyboard,
        #twist_stamper
        #robot_description
    ])