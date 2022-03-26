"""
ROS interface to world model
"""

from rclpy.node import Node

from pyrobosim.msg import TaskAction


class WorldROSWrapper(Node):
    def __init__(self, world):
        super().__init__("world_ros_wrapper")

        # Connect the ROS node to the world
        self.world = world
        self.world.ros_node = self
        self.world.has_ros_node = True

        # Subscriber to single action
        self.action_sub = self.create_subscription(
            TaskAction, "/commanded_action", self.action_callback, 1)

        print("World ROS node started")


    def action_callback(self, msg):
        """ Handle single action callback """
        self.get_logger().info(f"Executing action {msg.type}")
        if msg.type == "navigate":
            if self.world.has_gui:
                self.world.gui.navigate(msg.target_location)
            else:
                path = self.world.find_path(msg.target_location)
                self.world.execute_path(path, dt=0.1, realtime_factor=1.0,
                                        linear_velocity=1.0, max_angular_velocity=None)
        elif msg.type == "pick":
            if self.world.has_gui:
                self.world.gui.pick_object(None)
            else:
                self.world.pick_object(None)
        elif msg.type == "place":
            if self.world.has_gui:
                self.world.gui.place_object(None)
            else:
                self.world.place_object(None)
