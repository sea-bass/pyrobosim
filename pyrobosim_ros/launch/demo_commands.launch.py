from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value="",
        description="YAML file name (should be in the pyrobosim/data folder). "
        + "If not specified, a world will be created programmatically.",
    )
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="plan",
        description="Command mode (action or plan)",
    )
    action_delay_arg = DeclareLaunchArgument(
        "action_delay",
        default_value="0.0",
        description="The action delay, in seconds",
    )
    action_success_probability_arg = DeclareLaunchArgument(
        "action_success_probability",
        default_value="1.0",
        description="The action success probability, in the range (0, 1)",
    )
    action_rng_seed_arg = DeclareLaunchArgument(
        "action_rng_seed",
        default_value="-1",
        description="The random number generator seed. Defaults to -1, or nondeterministic.",
    )
    send_cancel_arg = DeclareLaunchArgument(
        "send_cancel",
        default_value="False",
        description="If True, cancels running actions after some time.",
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
        parameters=[
            {
                "mode": LaunchConfiguration("mode"),
                "action_delay": LaunchConfiguration("action_delay"),
                "action_success_probability": LaunchConfiguration(
                    "action_success_probability"
                ),
                "action_rng_seed": LaunchConfiguration("action_rng_seed"),
                "send_cancel": LaunchConfiguration("send_cancel"),
            }
        ],
    )

    return LaunchDescription(
        [
            world_file_arg,
            mode_arg,
            action_delay_arg,
            action_success_probability_arg,
            action_rng_seed_arg,
            send_cancel_arg,
            world_node,
            command_node,
        ]
    )
