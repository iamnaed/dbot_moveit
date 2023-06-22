from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Initialize moveit_config
    moveit_config = MoveItConfigsBuilder("dbot_world", package_name="dbot_moveit_config").to_moveit_configs()

    # MoveGroupInterface demo executable
    dbot_moveit_node = Node(
        name="dbot_moveit",
        package="dbot_moveit",
        executable="dbot_moveit",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([dbot_moveit_node])