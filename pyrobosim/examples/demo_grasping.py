#!/usr/bin/env python3

from pyrobosim.manipulation.grasping import GraspGenerator, ParallelGraspProperties

if __name__ == "__main__":
    obj_dims = [0.1, 0.05, 0.2]

    properties = ParallelGraspProperties(
        max_width=0.15, depth=0.1, height=0.04
    )
    gen = GraspGenerator(properties)

    grasps = gen.generate(obj_dims)
    print(grasps)

    gen.show_grasps(obj_dims, grasps)
