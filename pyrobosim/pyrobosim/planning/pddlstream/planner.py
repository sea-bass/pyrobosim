""" Task and Motion Planning tools using PDDLStream. """

import os
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


class PDDLStreamPlanner:
    """Task and Motion Planner using PDDLStream."""

    def __init__(
        self,
        world,
        domain_folder,
        stream_map_fn=get_default_stream_map_fn(),
        stream_info_fn=get_default_stream_info_fn(),
    ):
        """
        Creates a new PDDLStream based planner.

        :param world: World object to use for planning.
        :type world: :class:`pyrobosim.core.world.World`
        :param domain_folder: Path to folder containing PDDL domain and streams
        :type domain_folder: str
        :param stream_map_fn: Function that accepts a World and Robot object and returns a dictionary of stream mappings.
        :type stream_map_fn: function
        :param stream_info_fn: Function that returns a dictionary of stream information.
        :type stream_info_fn: function
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
        self.latest_specification = None
        self.latest_plan = None

    def plan(
        self,
        robot,
        goal_literals,
        max_attempts=1,
        verbose=False,
        **planner_config,
    ):
        r"""
        Searches for a set of actions that completes a goal specification
        given an initial state of the world.
        This uses the "adaptive" planner in PDDLStream, which demonstrates the best performance
        for most problems.

        :param robot: Robot to use for planning.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param goal_literals: List of literals describing a goal specification.
        :type goal_literals: list[tuple]
        :param max_attempts: Maximum planning attempts.
        :type max_attempts: int, optional
        :param verbose: If True, prints additional information. Defaults to False.
        :type verbose: bool, optional
        :param \*\*planner_config: Additional keyword arguments to pass to the PDDLStream planner.
        :return: A task plan object ready to use with ``pyrobosim``.
        :rtype: :class:`pyrobosim.planning.actions.TaskPlan`
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
        constant_map = {}
        prob = PDDLProblem(
            self.domain_pddl,
            constant_map,
            external_pddl,
            self.stream_map_fn(self.world, robot),
            init,
            goal,
        )

        for i in range(max_attempts):
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
            plan_out = pddlstream_solution_to_plan(solution, robot.name)
            if plan_out is not None:
                break

        # Convert the solution to a TaskPlan object.
        self.latest_plan = plan_out
        if verbose:
            print("\nInitial conditions:")
            for i in init:
                print(f"\t{i}")
            print("\nGoal Specification:")
            for g in goal_literals:
                print(f"\t{g}")
            print("")
            print(self.latest_plan)
        return self.latest_plan
