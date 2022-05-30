Concepts
========

pyrobosim is primarily a world modeling framework for prototyping robotics applications.


Worlds
------

Worlds in pyrobosim consist of a hierarchy of polygonal *entities*, including:

* **Robots**: A movable entity capable of actions that can change its own state and the state of the world.
* **Rooms**: Regions that a robot can navigate.
* **Hallways**: Regions connecting two rooms, which a robot can also navigate.
* **Locations**: Regions inside rooms that may contain objects (e.g., furniture).
* **Object Spawns**: Subregions of locations where objects may exist (e.g., a left vs. right countertop).
* **Objects**: Discrete entities that can be manipulated around the world.

This is all represented in a 2.5D environment (SE(2) pose with vertical (Z) height).

.. image:: media/world_entities.png
    :align: center
    :width: 600px
    :alt: Entities in a world.

|

Actions
-------

Within the world, we can spawn a robot that can perform a set of *actions*, such as:

* **Navigate** to a particular entity or pose.
* **Pick** an object from a specific location.
* **Place** an object at a specific location and pose.

These actions can be specified individually, or a sequence of actions (or a *plan*).
Actions or plans can be commanded directly (e.g., "go to the table and pick up an apple")
or as part of a *task and motion planning* framework that accepts a task specification 
(e.g., "all apples should be on the kitchen table") and outputs a plan that, when executed,
satisfies the specification.

For example, here is a robot performing a **navigate** action from the kitchen to the desk
in our simple test world.

.. image:: media/example_navigate.png
    :align: center
    :width: 600px
    :alt: Example navigation action.

|

Vision Statement
----------------
The vision for pyrobosim is that you will be able to **create worlds** to prototype your 
robot behavior in a simple environment before moving to a more realistic simulator, or even
real robot hardware.

To enable this, a typical user of pyrobosim would:

* **Build complex worlds** using the world modeling framework, both manually and programmatically.
* **Define custom actions and action executors** (e.g. path planning/following or decision-making algorithms).
* **Design task and motion planners** that go from task specification to task plan.
* **Export worlds to Gazebo** to test in a more photorealistic 3D world with a better robot model.

Some use cases include:

* `Home Service Robotics with the Toyota HSR <https://roboticseabass.com/2020/12/30/2020-review-service-robotics-mit-csail/>`_ -- the start of this work!

We look forward to your Git issues, contributions, and usage examples!
