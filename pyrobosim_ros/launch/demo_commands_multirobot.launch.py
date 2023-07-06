from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Nodes
    world_node = Node(
        package="pyrobosim_ros",
        executable="demo.py",
        name="demo_world",
        parameters=[
            {
                # Use multirobot file option.
                "world_file": "test_world_multirobot.yaml"
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

    return LaunchDescription([world_node, command_node])
