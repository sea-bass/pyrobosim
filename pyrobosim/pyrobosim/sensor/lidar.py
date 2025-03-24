import numpy as np
import itertools
import time

from shapely import intersects_xy
from ..utils.pose import Pose

class Lidar():
    """
    Implements a lidar sensor to detect collision.
    """

    def __init__(
        self,
        scan_radius: float = 0.3,
        xy_step_distance: float = 0.1,
        ignore_robots: bool = True,
    ) -> None:
        from ..core.robot import Robot
        self.robot: Robot | None = None

        self.scan_radius = scan_radius
        self.xy_step_distance = xy_step_distance
        self.ignore_robots = ignore_robots

        self.scan_poses: list[Pose] | None = []

    def scan(self) -> None:
        """
        Scan around robot and update self.scan_poses.
        """

        cur_pose = self.robot.get_pose()

        scan_points = []

        # Collect sampled points using grid-based sampling method
        for dx in np.arange(-self.scan_radius, (self.scan_radius + self.xy_step_distance), self.xy_step_distance):
            for dy in np.arange(-self.scan_radius, (self.scan_radius + self.xy_step_distance), self.xy_step_distance):
                px, py = cur_pose.x + dx, cur_pose.y + dy

                # Check if the point is within the room/hallway internal collision polygon
                # And check if the point is within the scan radius - (computed px,py from above loop would form a rectangle)
                for entity in itertools.chain(self.robot.world.rooms, self.robot.world.hallways):
                    has_close_polygon = hasattr(entity, 'closed_polygon')
                    if has_close_polygon:
                        if intersects_xy(entity.internal_collision_polygon, px, py) or intersects_xy(entity.closed_polygon, px, py):
                            if(px-cur_pose.x)**2 + (py-cur_pose.y)**2 <= self.scan_radius**2:
                                scan_points.append((px, py))
                    elif not has_close_polygon:
                        if intersects_xy(entity.internal_collision_polygon, px, py):
                            if(px-cur_pose.x)**2 + (py-cur_pose.y)**2 <= self.scan_radius**2:
                                scan_points.append((px, py))

        self.scan_poses = []

        for point in scan_points:
            self.scan_poses.append(Pose(x=point[0], y=point[1]))


    def get_scan_poses(self) -> list[Pose] | None:
        """
        Returns the points that the lidar sensor detects.
        
        :return: List of scanned poses.
        """
        return self.scan_poses
        

    def detect_collision(self) -> bool:
        """
        Carry out lidar sensing, and examine if scanned poses collide with any objects.

        *This would be run as a thread for now.

        :return: True if collision is detected, else False.
        """

        self.scan()
        
        for pose in self.scan_poses:

            # Check if the pose is occupied
            # Return True (Detected collision) if we verify a pose is occupied
            if self.robot.world.check_occupancy(pose) or (
                not self.ignore_robots and self.robot.world.collides_with_robots(pose, self.robot)):
                return True
            

            # If we go through all hallways and cant verify pose is occupied,
            # return False (No collision)

        return False

