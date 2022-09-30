""" Task and Motion Planning tools using PDDLStream. """

import os
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import And, PDDLProblem
from pddlstream.utils import read

from .mappings import get_stream_info, get_stream_map
from .utils import process_goal_specification, world_to_pddlstream_init, \
    pddlstream_solution_to_plan


class PDDLStreamPlanner:
    """Task and Motion Planner using PDDLStream."""

    def __init__(self, world, domain_folder):
        """
        Creates a new PDDLStream based planner.

        :param world: World object to use for planning.
        :type world: :class:`pyrobosim.core.world.World`
        :param domain_folder: Path to folder containing PDDL domain and streams
        :type domain_folder: str
        """
        # Set the world model
        self.world = world

        # Configuration parameters
        domain_pddl_file = os.path.join(domain_folder, "domain.pddl")
        self.domain_pddl = read(domain_pddl_file)
        stream_pddl_file = os.path.join(domain_folder, "streams.pddl")
        self.stream_pddl = read(stream_pddl_file)

        # Bookkeeping variables
        self.latest_specification = None
        self.latest_plan = None

    def plan(
        self,
        robot,
        goal_literals,
        focused=True,
        planner="ff-astar",
        max_time=60.0,
        max_iterations=10,
        search_sample_ratio=1.0,
        verbose=False,
    ):
        """
        Searches for a set of actions that completes a goal specification
        given an initial state of the world.

        :param robot: Robot to use for planning.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param goal_literals: List of literals describing a goal specification.
        :type goal_literals: list[tuple]
        :param focused: If True (default), uses the focused algorithm; else, uses incremental.
        :type focused: bool, optional
        :param planner: Planner used by PDDLStream, defaults to ``ff-astar``.
        :type planner: str, optional
        :param max_time: Max planning time.
        :type max_time: float, optional
        :param max_iterations: Maximum planning iterations.
        :type max_iterations: int, optional
        :param search_sample_ratio: Search to sample time ratio, used only for the focused algorithm.
        :type search_sample_ratio: float, optional
        :param verbose: If True, prints additional information. Defaults to False.
        :type verbose: bool, optional
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
            get_stream_map(self.world, robot),
            init,
            goal,
        )

        # Solve the problem using either focused or incremental algorithms.
        # The ``get_stream_info()`` function comes from the ``mappings.py``
        # file, so as you add new functionality you should fill it out there.
        if focused:
            solution = solve_focused(
                prob,
                planner=planner,
                stream_info=get_stream_info(),
                search_sample_ratio=search_sample_ratio,
                max_time=max_time,
                max_planner_time=max_time,
                max_iterations=max_iterations,
                initial_complexity=0,
                verbose=verbose,
            )
        else:
            solution = solve_incremental(
                prob, planner=planner, max_time=max_time, verbose=verbose
            )

        # Convert the solution to a TaskPlan object.
        plan_out = pddlstream_solution_to_plan(solution, robot.name)
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
