from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Load Moveit
    moveit_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('case_moveit_config'), 'launch', 'demo.launch.py'])
            ),
        )
    
    #Robot Logic Node
    robot_logic_launch = TimerAction(
            period=6.0,  # Staggered startup for stability
            actions=[
                Node(
                        package='case_task',
                        executable='robot_logic',
                        output='screen'
                    )
            ]
        )

    return LaunchDescription([
            moveit_launch,
            robot_logic_launch
        ])