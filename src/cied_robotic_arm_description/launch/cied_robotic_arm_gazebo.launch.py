import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

description_pkg = "cied_robotic_arm_description"
xacro_filename = "cied_robotic_arm.xacro"
def generate_launch_description():

    # Path to xacro file
    xacro_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', xacro_filename)
    # model_arg
    model_args = DeclareLaunchArgument(
        name = "model",
        default_value = xacro_file,
        description = "Absolute path to robot urdf"
    )

    # Environment Variable
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_prefix(description_pkg), "share")

    # robot_description
    robot_description = ParameterValue(Command(
            ['xacro ', LaunchConfiguration("model")]
        )
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        arguments=[xacro_file],
    )

    # gazebo
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    #     )
    # )

    start_gazebo_server_cmd = ExecuteProcess(
       cmd=[
            'gzserver',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',

        ],        
        output='screen',
    )
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=[
            'gzclient',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',

        ],
        output='screen',
    )
    
    # gazebo spawn entity
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "cied_robotic_arm", "-topic", "robot_description"],
        output="screen",
    )

            

    return LaunchDescription([
        model_args,
        robot_state_publisher,
        # gazebo,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,  # Uncomment when gazebo is running in the background.
        spawn_entity,
    ])