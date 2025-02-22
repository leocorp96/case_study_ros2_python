from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = FindPackageShare('case_description')
    pkg_path_gz = FindPackageShare('case_gz')
    control_config = PathJoinSubstitution([pkg_path_gz, 'config', 'ros2_control.yaml'])

    # Include the existing launch file that loads robot_state_publisher
    robot_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_path, 'launch', 'display.launch.py'])
            ),
            launch_arguments={
            'gui': 'false'}.items()
        )
    # Load and start the controller manager
    controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[control_config],
            output='screen'
        )
    # Delay controller_manager startup by 3 seconds
    delayed_controller_manager = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[controller_manager_node]
    )

    # Load Gazebo Sim
    # gz_sim_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution([
    #                 FindPackageShare('ros_gz_sim'), 'launch', 'ros_gz_spawn_model.launch.py'])
    #         ),
    #         launch_arguments={
    #         'file': 'empty.sdf'}.items()
    #     )

    # Define the controllers to be spawned
    controllers = ["arm_controller", "screw_controller", "joint_state_broadcaster"]

    # Spawn controllers using TimerActions with delays
    spawn_controllers = [
        TimerAction(
            period=4.0 + i,  # Staggered startup for stability
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[controller, '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        )
        for i, controller in enumerate(controllers)
    ]

    return LaunchDescription([
        #gz_sim_launch,
        robot_description_launch,
        delayed_controller_manager,

        *spawn_controllers # Unpack the list of controller spawners
    ])