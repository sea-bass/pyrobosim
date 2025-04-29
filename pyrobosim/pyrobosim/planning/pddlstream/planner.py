"""Task and Motion Planning tools using PDDLStream."""

import os
from typing import Any, Callable

from pddlstream.algorithms.focused import solve_adaptive
from pddlstream.language.constants import And, PDDLProblem
from pddlstream.utils import read

from .utils import (
    get_default_stream_info_fn,
    get_default_stream_map_fn,
    process_goal_specification,
    world_to_pddlstream_init,
    pddlstream_solution_to_plan,
)
from ..actions import TaskPlan
from ...core.robot import Robot
from ...core.types import Entity
from ...core.world import World


class PDDLStreamPlanner:
    """Task and Motion Planner using PDDLStream."""

    def __init__(
        self,
        world: World,
        domain_folder: str,
        stream_map_fn: Callable[
            [World, Robot], dict[str, Any]
        ] = get_default_stream_map_fn(),
        stream_info_fn: Callable[[], dict[str, Any]] = get_default_stream_info_fn(),
    ) -> None:
        """
        Creates a new PDDLStream based planner.

        :param world: World object to use for planning.
        :param domain_folder: Path to folder containing PDDL domain and streams
        :param stream_map_fn: Function that accepts a World and Robot object and returns a dictionary of stream mappings.
        :param stream_info_fn: Function that returns a dictionary of stream information.
        """
        # Set the world model
        self.world = world

        # Planning configuration parameters
        domain_pddl_file = os.path.join(domain_folder, "domain.pddl")
        self.domain_pddl = read(domain_pddl_file)
        stream_pddl_file = os.path.join(domain_folder, "streams.pddl")
        self.stream_pddl = read(stream_pddl_file)
        self.stream_map_fn = stream_map_fn
        self.stream_info_fn = stream_info_fn

        # Bookkeeping variables
        self.latest_specification: list[tuple[str | Entity, ...]] | None = None
        self.latest_plan: TaskPlan | None = None

    def plan(
        self,
        robot: Robot,
        goal_literals: list[tuple[str | Entity, ...]],
        max_attempts: int = 1,
        verbose: bool = False,
        **planner_config: Any,
    ) -> TaskPlan | None:
        r"""
        Searches for a set of actions that completes a goal specification
        given an initial state of the world.
        This uses the "adaptive" planner in PDDLStream, which demonstrates the best performance
        for most problems.

        :param robot: Robot to use for planning.
        :param goal_literals: List of literals describing a goal specification.
        :param max_attempts: Maximum planning attempts.
        :param verbose: If True, prints additional information. Defaults to False.
        :param \*\*planner_config: Additional keyword arguments to pass to the PDDLStream planner.
        :return: A task plan object ready to use with PyRoboSim.
        """
        # Set the initial and goal states for PDDLStream
        init = world_to_pddlstream_init(self.world, robot)
        process_goal_specification(goal_literals, self.world)
        goal = And(*goal_literals)
        self.latest_specification = goal

        # Set up the PDDL problem.
        # The ``get_stream_map()`` function comes from the ``mappings.py``
        # file, so as you add new functionality you should fill it out there.
        external_pddl = [self.stream_pddl]
        prob = PDDLProblem(
            self.domain_pddl,
            {},  # constant_map
            external_pddl,
            self.stream_map_fn(self.world, robot),
            init,
            goal,
        )

        for _ in range(max_attempts):
            # Solve the problem using the "adaptive" PDDLStream algorithm.
            solution = solve_adaptive(
                prob,
                stream_info=self.stream_info_fn(),
                verbose=verbose,
                **planner_config,
            )

            # If the solution is valid, no need to try again
            # TODO: Could later consider an option to execute all the attempts
            # and take the minimum-cost plan from that batch.
            self.latest_plan = pddlstream_solution_to_plan(solution, robot.name)
            if self.latest_plan is not None:
                break

        # Convert the solution to a TaskPlan object.
        if verbose:
            verbose_str = "\nInitial conditions:\n"
            for i in init:
                verbose_str += f"\t{i}\n"
            verbose_str += "\nGoal Specification:\n"
            for g in goal_literals:
                verbose_str += f"\t{g}\n"
            verbose_str += f"\n{self.latest_plan}"
            robot.logger.info(verbose_str)
        return self.latest_plan
