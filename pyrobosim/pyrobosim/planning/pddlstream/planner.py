import os

from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.constants import And, PDDLProblem
from pddlstream.language.function import FunctionInfo
from pddlstream.utils import read

from . import primitives
from .utils import world_to_pddlstream_init, pddlstream_solution_to_plan

class PDDLStreamPlanner():
    """ Task and Motion Planner using PDDLStream """

    def __init__(self, world):
        """
        Creates a new planner.
        """
        
        # Set the world model
        self.world = world
        
        # Configuration parameters
        domains_folder = os.path.join(os.path.split(__file__)[0], "domains")
        domain_pddl_file = os.path.join(domains_folder, "domain_simple.pddl")
        stream_pddl_file = os.path.join(domains_folder, "stream_simple.pddl")
        self.domain_pddl = read(domain_pddl_file)
        self.stream_pddl = read(stream_pddl_file)

        # Stream functions
        pick_place_cost = 0.5
        self.pick_place_cost_fn = lambda l, o: pick_place_cost

        # Parameters
        self.latest_specification = None
        self.latest_plan = None


    def plan(self, goal_literals, focused=False, planner="ff-astar", 
             max_time=60, verbose=False):
        """
        Searches for a set of actions that completes a goal specification 
        given an initial state of the world.
        """
        # Set the initial and goal states for PDDLStream
        init = world_to_pddlstream_init(self.world)
        goal = And(*goal_literals)
        self.latest_specification = goal

        # Set up the PDDL problem
        # TODO: Automatically go through some other file to fill this in.
        stream_map = {"dist": primitives.get_distance,
                      "pickplacecost": self.pick_place_cost_fn
                     }
        stream_info = {"dist": FunctionInfo(opt_fn=primitives.get_distance),
                       "pickplacecost": FunctionInfo(opt_fn=self.pick_place_cost_fn)
                      }
        external_pddl = [self.stream_pddl]
        constant_map = {}
        prob = PDDLProblem(self.domain_pddl, 
                          constant_map,
                          external_pddl, 
                          stream_map,
                          init,
                          goal)

        # Solve the problem
        if focused:
            solution = solve_focused(prob,
                planner=planner,
                stream_info=stream_info,
                search_sample_ratio=1.0,
                max_time=max_time,
                max_planner_time=max_time,
                max_iterations=10,
                initial_complexity=0,
                verbose=verbose
            )
        else:
            solution = solve_incremental(prob,
                planner=planner,
                max_time=max_time,
                verbose=verbose
            )

        # Convert the solution to a plan list
        plan_out = pddlstream_solution_to_plan(solution)
        self.latest_plan = plan_out
        return self.latest_plan
