from typing import Any
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def launch_planner_node(
    context: LaunchContext, *args: Any, **kwargs: Any
) -> list[Node]:
    verbose_bool = LaunchConfiguration("verbose").perform(context).lower() == "true"
    planner_node = Node(
        package="pyrobosim_ros",
        executable="demo_pddl_planner.py",
        name="pddl_demo_planner",
        parameters=[
            {
                "example": LaunchConfiguration("example"),
                "subscribe": LaunchConfiguration("subscribe"),
                "verbose": LaunchConfiguration("verbose"),
                "search_sample_ratio": LaunchConfiguration("search_sample_ratio"),
            }
        ],
        output="screen",
        emulate_tty=verbose_bool,
    )
    return [planner_node]


def generate_launch_description() -> LaunchDescription:
    # Arguments
    example_arg = DeclareLaunchArgument(
        "example",
        default_value=TextSubstitution(text="01_simple"),
        description="Example name, must be one of "
        + "(01_simple, 02_derived, 03_nav_stream, 04_nav_manip_stream, 05_nav_grasp_stream, 06_open_close_detect)",
    )
    verbose_arg = DeclareLaunchArgument(
        "verbose",
        default_value=TextSubstitution(text="true"),
        description="Print planner output (true/false)",
    )
    subscribe_arg = DeclareLaunchArgument(
        "subscribe",
        default_value=TextSubstitution(text="true"),
        description="If true, waits to receive goal on a subscriber.",
    )
    search_sample_ratio_arg = DeclareLaunchArgument(
        "search_sample_ratio",
        default_value=TextSubstitution(text="0.5"),
        description="Search to sample ratio for planner",
    )

    # Nodes
    world_node = Node(
        package="pyrobosim_ros",
        executable="demo_pddl_world.py",
        name="pddl_demo",
    )
    planner_node = OpaqueFunction(function=launch_planner_node)
    goalspec_node = Node(
        package="pyrobosim_ros",
        executable="demo_pddl_goal_publisher.py",
        name="pddl_demo_goal_publisher",
        parameters=[
            {
                "example": LaunchConfiguration("example"),
                "verbose": LaunchConfiguration("verbose"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("subscribe")),
    )

    return LaunchDescription(
        [
            example_arg,
            verbose_arg,
            subscribe_arg,
            search_sample_ratio_arg,
            world_node,
            planner_node,
            goalspec_node,
        ]
    )
