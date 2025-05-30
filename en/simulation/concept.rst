Basic Concepts
==============

.. contents::
   :local:
   :depth: 1


Simulation Functionality
------------------------

Choreonoid is equipped with simulation capabilities and can be used as a robot simulator. It simulates how robots and environmental objects move through calculations and displays the results as 3D CG animations or outputs them as data. Using this functionality, it becomes possible to verify hardware design and software without operating actual robots, as well as practice operations and procedures.

The targets of simulation are the "body models" introduced in :doc:`../handling-models/index`. By loading body models as body items and having them belong to a single virtual world through a world item, simulation of this virtual world becomes possible.


Physics Engine
--------------

The core part of a simulator is the component that calculates how objects physically move, which is called a "physics engine". Various algorithms for physics calculations have been devised, and many physics engines implementing these algorithms have been developed. The models and physical phenomena that can be simulated depend on the engine, and characteristics such as simulation accuracy, stability, and calculation speed also vary by engine. Therefore, it is important to use different engines according to the simulation target and purpose. From this perspective, Choreonoid is designed to utilize various physics engines.

.. _simulation_simulator_item:

Simulator Item
--------------

In Choreonoid, physics engines are represented as project items called "simulator items". These are designed to integrate physics engines into Choreonoid's simulation functionality and provide the foundational API for this purpose. What you actually use are items that inherit from simulator items, with corresponding simulator items defined for each physics engine. Specifically, the following simulator items are available:

* **AIST Simulator Item**

 Choreonoid's standard simulator item that performs simulation using its own physics calculation engine.


* **ODE Simulator Item**

 A simulator item that uses the open-source physics engine `Open Dynamics Engine (ODE) <http://www.ode.org/>`_. It becomes available by installing ODE and building the ODE Plugin.

* **AGX Simulator Item**

 A simulator item that uses the commercial physics engine `AGX Dynamics <https://www.algoryx.se/products/agx-dynamics/>`_ developed by Algoryx Simulation AB of Sweden. It becomes available through :doc:`../agxdynamics/index`.

* **Other Simulator Items**

 Support for physics engines `Bullet Physics Library <http://bulletphysics.org>`_ and `PhysX <https://developer.nvidia.com/gameworks-physx-overview>`_ is also being developed. However, these are currently experimental implementations.

.. note:: Physics calculations also require detecting interference between simulated objects, and collision detectors that perform this are typically included in physics calculation engines. On the other hand, as explained in :doc:`../handling-models/collision-detection` in :doc:`../handling-models/index`, there is also collision detection functionality provided as a basic feature of Choreonoid, with various collision detectors available (:ref:`handling-models_switch-collision-detector`). Some simulator items allow the use of any collision detector from the basic functionality.

.. _simulation_subsimulator:

Sub-simulator
-------------

In addition to simulator items that perform basic physics calculations, "sub-simulators" are also available as items to supplement these and implement various simulation features.

For example, you may want to simulate the functionality of cameras and range sensors mounted on robots to obtain camera images and range images during simulation. The :ref:`simulation-gl-vision-simulator` item is provided as a sub-simulator that adds this functionality. This sub-simulator simulates sensor output by internally performing rendering similar to 3D CG display from the viewpoint of cameras and range sensors. In contrast to the "physics engine", this could be called a "vision engine". Since this functionality does not depend on physics calculation algorithms, it can be used in combination with any simulator item.

Sub-simulators can implement various other features within the framework of monitoring the virtual world situation and providing corresponding output or modifying the virtual world.

.. _simulation_controller:

Controller
----------

To operate a robot, a control program is required, which is called a "controller". Controllers are also necessary in simulations to operate robots. Generally, the same controller is used for both operating actual robots and for simulations. This approach enables efficient development and verification of controllers on simulators. Additionally, this allows users of the developed robot system to practice its operation and procedures on the simulator.

In any case, controllers are necessary to operate robots and are one of the main components of simulation.

.. _simulation-concept-controller-item:

Controller Item
---------------

In Choreonoid's simulation functionality, controllers are represented as project items called "controller items". In practice, item types that inherit from this are used to operate controller bodies implemented separately from controller items. There are various formats for controller bodies, and if a controller item corresponding to a particular format is available, controllers in that format can be used.

For example, the :ref:`ros2_control_item` for using controllers from ROS2's control framework "ros2_control" is made available through the :doc:`../ros2/ros2-plugin`.

The usage of controller items is explained in :doc:`howto-use-controller`.


Input/Output Between Robot and Controller
-----------------------------------------

The first thing necessary for a controller to control a robot is to perform input/output of various data with the robot. That is, the controller first obtains the state of the robot and environment from input from various sensors mounted on the robot, performs control calculations based on this, and then outputs the determined command values to the robot's actuators.

Specifically, the following elements are targets for input:

* Joint angles of rotational joints
* Joint translation of linear joints
* Force sensors
* Acceleration sensors
* Angular acceleration sensors (rate gyros)
* Camera images
* Range sensor distance images

The following elements are targets for output:

* Torque applied to rotational joints
* Force applied to linear joints
* Operation commands for various devices (lights, etc.) (on/off, etc.)

You can think of controller items as defining the interfaces for these inputs and outputs.

The actual input/output methods are explained in :doc:`howto-implement-controller`.

Utilizing Plugins
-----------------

For simulator items, sub-simulator items, and controller items, their inherited item types can be added through plugins. This enables:

* Adding available physics engines
* Extending simulation functionality
* Adding supported controller formats

Choreonoid can be said to be a platform for extending simulation functionality itself in this way.

For information on implementing simulator items, :doc:`../plugin-development/ode-plugin` in :doc:`../plugin-development/index` may be helpful.