#!/usr/bin/env python3

"""
Tests for hallway creation in pyrobosim.
"""

from pyrobosim.core import Hallway, World


class TestHallway:
    def test_add_hallway_to_world_from_object(self):
        """Test adding a hallway from a Hallway object."""
        world = World()

        coords_start = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        room_start = world.add_room(name="room_start", footprint=coords_start)

        coords_end = [(3.0, 5.0), (5.0, 3.0), (5.0, 5.0), (3.0, 5.0)]
        room_end = world.add_room(name="room_end", footprint=coords_end)

        hallway = Hallway(room_start=room_start, room_end=room_end, width=0.1)
        result = world.add_hallway(hallway=hallway)
        assert isinstance(result, Hallway)
        assert world.num_hallways == 1
        assert world.hallways[0].room_start == room_start
        assert world.hallways[0].room_end == room_end
        assert world.hallways[0].width == 0.1

    def test_add_hallway_to_world_from_args(self):
        """Test adding a room from a list of named keyword arguments."""
        world = World()

        coords_start = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        room_start = world.add_room(name="room_start", footprint=coords_start)

        coords_end = [(3.0, 5.0), (5.0, 3.0), (5.0, 5.0), (3.0, 5.0)]
        room_end = world.add_room(name="room_end", footprint=coords_end)

        result = world.add_hallway(
            room_start=room_start, room_end="room_end", width=0.1
        )
        assert isinstance(result, Hallway)
        assert world.num_hallways == 1
        assert world.hallways[0].room_start == room_start
        assert world.hallways[0].room_end == room_end
        assert world.hallways[0].width == 0.1


if __name__ == "__main__":
    t = TestHallway()
    t.test_add_hallway_to_world_from_object()
    t.test_add_hallway_to_world_from_args()
    print("Tests passed!")
