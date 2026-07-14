from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Arguments
    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value="",
        description="YAML file name (should be in the pyrobosim/data folder). "
        + "If not specified, a world will be created programmatically.",
    )
    web_arg = DeclareLaunchArgument(
        "web",
        default_value="false",
        description="If true, launches the browser-based web UI instead of the Qt GUI.",
    )

    # Nodes
    demo_node = Node(
        package="pyrobosim_ros",
        executable="demo.py",
        name="demo",
        parameters=[
            {
                "world_file": LaunchConfiguration("world_file"),
                "web": LaunchConfiguration("web"),
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([world_file_arg, web_arg, demo_node])
