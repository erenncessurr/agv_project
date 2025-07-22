import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import xacro


def generate_launch_description():

    package_name='agv_project' 
    pkg_path = get_package_share_directory('agv_project_py')
    
    plc_odom = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_path, 'launch', 'plc_odom.launch.py')
    )
)

    twist_mux_params = os.path.join(get_package_share_directory('agv_project'),'config','twist_mux.yaml')
    
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel','/tricycle_controller/cmd_vel')]
        )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "description",
                    "robot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
         [
         FindPackageShare('agv_project'),
         'config',
         'controllers_tricycle.yaml',
         ]
     )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,robot_controllers],
        output="both"
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    joint_state_broadcaster_spawner = Node(
         package='controller_manager',
         executable='spawner',
         arguments=[
             'joint_state_broadcaster',
             '--param-file',
             robot_controllers,
             ],
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
    
    delayed_tricycle_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[tricycle_controller_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription([
            twist_mux,
            delayed_controller_manager,
            delayed_joint_broad_spawner,
            delayed_tricycle_controller_spawner,
            robot_state_pub_node,
            plc_odom
            ])