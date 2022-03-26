import rclpy
from rclpy.node import Node
from std_msgs.msg import String


"""
ROS interface to world model
"""

class WorldROSWrapper(Node):
    def __init__(self, world):
        super().__init__("world_ros_wrapper")

        # Connect the ROS node to the world
        self.world = world
        self.world.ros_node = self
        self.world.has_ros_node = True

        # TODO: Use actual TaskAction / TaskPlan interface
        self.publisher = self.create_subscription(
            String, "action", self.action_callback, 10)

        print("World ROS node started")


    def action_callback(self, msg):
        """ 
        Handle action callback 
        TODO: Fill in with actual TaskAction / TaskPlans
        """
        self.get_logger().info(f"Executing action {msg.data}")
        if msg.data == "go":
            import numpy as np
            loc = np.random.choice(self.world.locations)
            if self.world.has_gui:
                self.world.gui.navigate(loc)
            else:
                path = self.world.find_path(loc)
                self.world.execute_path(path, dt=0.05, realtime_factor=1.0,
                                        linear_velocity=1.0, max_angular_velocity=None)
        elif msg.data == "pick":
            if self.world.has_gui:
                self.world.gui.pick_object(None)
            else:
                self.world.pick_object(None)
        elif msg.data == "place":
            if self.world.has_gui:
                self.world.gui.place_object(None)
            else:
                self.world.place_object(None)
