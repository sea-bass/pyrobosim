from .robot import Robot

class World:
    def __init__(self, robot=Robot()):
        # Define a robot and all its sensors
        self.robot = robot

        # Define the world (rooms, locations, etc.)
        self.rooms = []
        self.locations = []
        self.hallways = []
        self.objects = []
