from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Arguments
    web_arg = DeclareLaunchArgument(
        "web",
        default_value="False",
        description="If True, launches the browser-based web UI instead of the Qt GUI.",
    )

    # Nodes
    world_node = Node(
        package="pyrobosim_ros",
        executable="demo.py",
        name="demo_world",
        parameters=[
            {
                # Use multirobot file option.
                "world_file": "test_world_multirobot.yaml",
                "web": LaunchConfiguration("web"),
            }
        ],
        output="screen",
        emulate_tty=True,
    )
    command_node = Node(
        package="pyrobosim_ros",
        executable="demo_commands.py",
        name="demo_commands",
        parameters=[
            {
                # Use multirobot plan mode option.
                "mode": "multirobot-plan"
            }
        ],
    )

    return LaunchDescription([web_arg, world_node, command_node])
