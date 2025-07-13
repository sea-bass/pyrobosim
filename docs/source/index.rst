.. PyRoboSim documentation master file.

PyRoboSim
=========

PyRoboSim is a ROS 2 enabled 2D mobile robot simulator for behavior prototyping.

.. youtube:: K5359uSeyVA

|

We look forward to your open-source contributions to PyRoboSim.
For more information, refer to the `contributor guide on GitHub <https://github.com/sea-bass/pyrobosim/blob/main/CONTRIBUTING.md>`_.


Vision Statement
----------------

The vision for PyRoboSim is that you will be able to **create worlds** to prototype your robot behavior in a simple environment before moving to a more realistic simulator, or even real robot hardware.

To enable this, a typical user of PyRoboSim would:

* **Build complex worlds** using the world modeling framework, both manually and programmatically.
* **Define custom actions and action executors** (e.g., path planning/following or decision-making algorithms).
* **Design task and motion planners** that go from task specification to an executable plan.
* **Export worlds to Gazebo** to test in a 3D world with more complex sensing and actuation models.


Usage Examples
--------------

Below is a list of known projects that use PyRoboSim.

* `Home Service Robotics at MIT CSAIL <https://roboticseabass.com/2020/12/30/2020-review-service-robotics-mit-csail/>`_, by Sebastian Castro (2020) -- the precursor to this work!
* `Task Planning in Robotics <https://roboticseabass.com/2022/07/19/task-planning-in-robotics/>`_ and `Integrated Task and Motion Planning in Robotics <https://roboticseabass.com/2022/07/30/integrated-task-and-motion-planning-in-robotics/>`_ blog posts, by Sebastian Castro (2022)
* `Hierarchically Decentralized Heterogeneous Multi-Robot Task Allocation System <https://arxiv.org/abs/2405.02484>`_ (2024) and `Evaluating the Impact of Resource Distribution on Efficiency of Multi-Robot Task Allocation System <https://ieeexplore.ieee.org/abstract/document/10976240>`_ (2025), by Sujeet Kashid and Ashwin D. Kumat
* `Hands-On with ROS 2 Deliberation Technologies <https://github.com/ros-wg-delib/roscon24-workshop>`_ workshop at `ROSCon 2024 <https://roscon.ros.org/2024/>`_, by Christian Henkel, Sebastian Castro, Davide Faconti, David Oberacker, David Conner, and Matthias Mayr (2024)
* `AutoAPMS: A unified software framework for creating behavior-based robotic applications <https://github.com/robin-mueller/auto-apms>`_, by Robin Müller (2025)
* `Collaborative Large Language Models for Task Allocation in Construction Robots <https://papers.ssrn.com/sol3/papers.cfm?abstract_id=5097309>`_, by Samuel A. Prieto and Borja García de Soto (2025)
* `Replacing Large Language Models for Personalized Robot Behavior <https://digital.wpi.edu/concern/etds/5999n813z>`_, by Tuomas Pyorre (2025)

If you have something to add, please submit a pull request!


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   concepts
   setup
   usage/index
   yaml/index
   api


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
