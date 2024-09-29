.. pyrobosim documentation master file.

pyrobosim
=========

``pyrobosim`` is a ROS 2 enabled 2D mobile robot simulator for behavior prototyping.

.. raw:: html

   <iframe width="560" height="315" src="https://www.youtube.com/embed/4UZOBplnTv8" title="pyrobosim demo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

|

We look forward to your open-source contributions to ``pyrobosim``.
For more information, refer to the `contributor guide on GitHub <https://github.com/sea-bass/pyrobosim/blob/main/CONTRIBUTING.md>`_.


Vision Statement
----------------

The vision for ``pyrobosim`` is that you will be able to **create worlds** to prototype your robot behavior in a simple environment before moving to a more realistic simulator, or even real robot hardware.

To enable this, a typical user of ``pyrobosim`` would:

* **Build complex worlds** using the world modeling framework, both manually and programmatically.
* **Define custom actions and action executors** (e.g., path planning/following or decision-making algorithms).
* **Design task and motion planners** that go from task specification to an executable plan.
* **Export worlds to Gazebo** to test in a 3D world with more complex sensing and actuation models.


Usage Examples
--------------

Below is a list of known projects that use ``pyrobosim``.

* `Home Service Robotics at MIT CSAIL <https://roboticseabass.com/2020/12/30/2020-review-service-robotics-mit-csail/>`_, by Sebastian Castro (2020) -- the precursor to this work!
* `Task Planning in Robotics <https://roboticseabass.com/2022/07/19/task-planning-in-robotics/>`_ and `Integrated Task and Motion Planning in Robotics <https://roboticseabass.com/2022/07/30/integrated-task-and-motion-planning-in-robotics/>`_ blog posts, by Sebastian Castro (2022)
* `Hierarchically Decentralized Heterogeneous Multi-Robot Task Allocation System <https://arxiv.org/abs/2405.02484>`_, by Sujeet Kashid and Ashwin D. Kumat (2024)
* `Hands-On with ROS 2 Deliberation Technologies <https://github.com/ros-wg-delib/roscon24-workshop>`_ workshop at `ROSCon 2024 <https://roscon.ros.org/2024/>`_, by Christian Henkel, Sebastian Castro, Davide Faconti, David Oberacker, David Conner, and Matthias Mayr (2024)

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
