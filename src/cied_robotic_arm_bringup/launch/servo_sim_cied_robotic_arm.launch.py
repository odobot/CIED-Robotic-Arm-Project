import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("cied_robotic_arm_description"),
                "launch",
                "cied_robotic_arm_gazebo.launch.py"
            )
        )
    
    controllers = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("cied_robotic_arm_controllers"),
                "launch",
                "cied_robotic_arm_controllers.launch.py"
            ),
            launch_arguments={"is_sim": "true"}.items()
        )
    
    servo_moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("cied_robotic_arm_moveit_servo"),
            "launch",
            "cied_robotic_arm_moveit_servo.launch.py"
        )
    )
    
    
    return LaunchDescription([
        gazebo,
        controllers,
        servo_moveit,
    ])