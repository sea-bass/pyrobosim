import itertools

from shapely.geometry import Point
from ..utils.pose import Pose

class Lidar():
    """
    Implements a lidar sensor to detect collision.
    """

    def __init__(
        self,
        scan_radius: float = 3.0,
        xy_step_distance: float = 0.125,
        ignore_robots: bool = True,
    ) -> None:
        from ..core.robot import Robot
        self.robot: Robot | None = None

        self.scan_radius = scan_radius
        self.xy_step_distance = xy_step_distance
        self.ignore_robots = ignore_robots

    def detect_collision(self) -> bool:
        """
        Detects collision using the lidar sensor.

        This would be run as a thread for now.

        :return: True if collision is detected, else False.
        """

        cur_pose = self.robot.get_pose()

        scan_points = []

        # Collect sampled points using grid-based sampling method
        for dx in range(int(-self.scan_radius), int(self.scan_radius) + self.xy_step_distance, self.xy_step_distance):
            for dy in range(int(-self.scan_radius), int(self.scan_radius) + self.xy_step_distance, self.xy_step_distance):
                px, py = cur_pose.x + dx, cur_pose.y + dy

                # Check if the point is within the room/hallway inernal collision polygon
                # And check if the point is within the scan radius ( computed px,py from above loop would form a rectangle)
                for entity in itertools.chain(self.robot.world.rooms, self.robot.world.hallways):
                    if entity.internal_collision_polygon.contains(Point(px, py)):
                        if(px-cur_pose.x)**2 + (py-cur_pose.y)**2 <= self.scan_radius**2:
                            scan_points.append((px, py))
        
        for point in scan_points:
            pose = Pose(x=point[0], y=point[1])
            if self.robot.world.check_occupancy(pose) or (
                not self.ignore_robots and self.robot.world.collides_with_robots(pose, self.robot)):
                return True

        return False

