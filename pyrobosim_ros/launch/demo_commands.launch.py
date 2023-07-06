from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value=TextSubstitution(text=""),
        description="YAML file name (should be in the pyrobosim/data folder). "
        + "If not specified, a world will be created programmatically.",
    )
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value=TextSubstitution(text="plan"),
        description="Command mode (action or plan)",
    )

    # Nodes
    world_node = Node(
        package="pyrobosim_ros",
        executable="demo.py",
        name="demo_world",
        parameters=[{"world_file": LaunchConfiguration("world_file")}],
    )
    command_node = Node(
        package="pyrobosim_ros",
        executable="demo_commands.py",
        name="demo_commands",
        parameters=[{"mode": LaunchConfiguration("mode")}],
    )

    return LaunchDescription([world_file_arg, mode_arg, world_node, command_node])
