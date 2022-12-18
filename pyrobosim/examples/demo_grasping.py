#!/usr/bin/env python3

from pyrobosim.manipulation.grasping import GraspGenerator

if __name__ == "__main__":
    obj_dims = [1, 1, 1]

    gen = GraspGenerator()

    grasps = gen.generate(obj_dims)
    print(grasps)

    gen.show_grasps(obj_dims, grasps)
