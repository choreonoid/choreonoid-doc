Competition Overview
====================

This section provides an overview of the WRS2018 robot competition "Tunnel Disaster Response and Recovery Challenge."

.. contents::
   :local:

Event Information
-----------------

This competition assumes disasters or accidents occurring within tunnels and challenges teams to demonstrate their robots' task execution capabilities in such situations. The competition consists of six tasks in total, with robots performing these tasks in a computer-simulated virtual environment.

The event overview is as follows:

* Dates: October 17 (Wed) - 21 (Sun), 2018
* Venue: Tokyo Big Sight
* Participation: Teams formed by interested participants apply through documentation. Participation is free. Teams can participate after passing the document screening.
* Number of participating teams: 8 teams in total
* `Prize money <http://worldrobotsummit.org/download/guideline/prize_money_for_the_wrc2018_en.pdf>`_: 1st place: 10 million yen, 2nd place: 3 million yen, 3rd place: 1 million yen

.. _wrs2018_overview_rules:

Competition Rules
-----------------

The competition rules are compiled in the following documents:

* :download:`Official English rulebook <rulebook-Tunnel_Disaster_Response_and_Recovery_Challenge.pdf>`
* :download:`Japanese reference version <detailed-rules-tunnel-disaster-response-and-recovery-challenge-ja.pdf>`

.. _wrs2018_overview_simulator:

Simulator Used
--------------

The following summarizes information about the simulator used in this competition:

* Simulator software: Choreonoid
* Choreonoid version: Development version used. The actual version used is `tagged as wrs2018 <https://github.com/choreonoid/choreonoid/tree/wrs2018>`_.
* OS: Ubuntu Linux 16.04 64-bit
* Utilizes the commercial physics engine "AGX Dynamics"

.. note:: This information reflects the conditions at the time of the competition. The WRS2018 samples can also run on Ubuntu 18.04 and 20.04 environments.

The development version of Choreonoid is available from the `Choreonoid repository on GitHub <https://github.com/choreonoid/choreonoid>`_.

The OS used is Ubuntu Linux, specifically the 64-bit version of 16.04.

This competition uses `AGX Dynamics <https://www.algoryx.se/products/agx-dynamics/>`_ as its physics engine. AGX Dynamics is a commercial physics engine requiring a license for use, but participants registered for this competition receive a free license for participation. With a license, you can use it on Choreonoid through the :doc:`../agxdynamics/index` included in Choreonoid itself.

.. note:: Even those not participating in the competition can try the competition simulations by purchasing the `AGX Dynamics license for Choreonoid <https://www.vmc-motion.com/商品・サービス/プラグインソリューション/plugin-for-choreonid/>`_ sold by `VMC Motion Technologies Co., Ltd. <https://www.vmc-motion.com/>`_. (Of course, it can also be used for simulations other than the competition.) Alternatively, you can get an overview of the competition using Choreonoid's standard physics engine without AGX Dynamics. However, in that case, robot and task simulations will be partially incomplete, and simulation speed will be slower.

The simulation PCs used at the competition venue are provided by the organizers with the following specifications:

* CPU: Intel Core i7 8700K (6 cores, 12 threads, base clock frequency 3.7GHz)
* Memory: 32GB
* GPU: NVIDIA GeForce GTX 1080 Ti

For development and testing for competition participation, participants need to use their own PCs. While having the same specifications as above would be ideal, the following specifications should generally be sufficient:

* CPU: Intel Core series, AMD Ryzen series, 4 or more cores, base frequency around 3GHz or higher
* GPU: NVIDIA GeForce/Quadro, Intel HD Graphics (CPU integrated)
* Memory: Around 8GB

Of course, simulation speed and rendering frame rates will vary depending on CPU and GPU specifications. Also, even with lower specifications than those listed above, the system will still run, just more slowly.

Please avoid using AMD GPUs (Radeon, etc.) as their Linux drivers may not be adequately maintained.

.. _wrs2018_overview_robots:

Target Robots
-------------

The following robot models are prepared for use in the competition:

.. image:: images/wrs-robots.png

Here's an overview of each robot:

* WAREC-1

 A robot developed primarily by Waseda University as part of the `ImPACT Tough Robotics Challenge (TRC) <http://www.jst.go.jp/impact/en/program/07.html>`_. This is a type of legged robot characterized by four legs arranged symmetrically around the torso. This allows it to be used as a quadruped robot, or by standing on two legs and using the other two as arms, it can function like a humanoid robot. Thus, it has the potential to handle various tasks in different configurations with creative approaches.

* Double-Arm Robot

 A robot developed primarily by Osaka University as part of ImPACT-TRC. This is a construction equipment-type robot equipped with two arms. Using these two arms and their attached end effectors, it can perform various tasks. Being construction equipment, it can handle tasks requiring significant force. It features crawlers for mobility, enabling movement on rough terrain. Furthermore, by utilizing both arms, it can enhance its rough terrain traversal capabilities and perform stable work on unstable footing.

* Aizu Spider

 A robot jointly developed by the University of Aizu and AISAC Corporation. This crawler-type robot features main crawlers plus auxiliary crawlers called flippers at the front and rear. It also includes a working arm. Robots of this form have seen increasing use as disaster response robots in recent years, holding great potential for investigation and work at disaster sites. This robot comes in three arm configurations: no arm, single arm, and dual arm. For tackling WRS tasks, the dual-arm type is the most practical choice.

WAREC-1 and Double-Arm Robot are official WRS :download:`platform robots <platformrobots.pdf>`. While Aizu Spider is not officially WRS-sanctioned, it was originally provided as a Choreonoid sample model and can be utilized in this competition. All these robots have actual physical counterparts that have been developed and are operational.

This competition also allows the use of multicopters (drones). As a multicopter model, the following quadrotor sample model developed by the Japan Atomic Energy Agency (JAEA) is available:

.. image:: images/quadcopter.png

Using the :doc:`../multicopter/index`, flight simulations of such multicopter models can be performed. In the competition, multicopters are expected to be used in conjunction with other robots to investigate tunnel conditions or provide overhead views for other robots' operations.

The above robot models come bundled with Choreonoid and can be used as-is in this competition. We'll refer to these as the "standard robots" for this competition.

Note that participants may also use robot models other than the standard robots. They may modify parts of the standard robots, use other Choreonoid sample models, or use their own unique robot models. However, such models must pass review by the competition organizing committee. The review criteria require that the robot either exists in reality or has a structure and specifications that could plausibly exist.

Task Overview
-------------

This competition is set in a scenario where "a disaster has occurred in a tunnel, causing wall collapses and vehicle accidents that scatter debris and create fires, making it dangerous for humans to enter. Robots are deployed into the tunnel to investigate conditions, rescue victims, and conduct firefighting activities." Under this premise, the competition consists of six specific tasks labeled T1 through T6, as shown in the figure below. The competition is conducted separately for each task.

.. image:: images/sixtaskimages.png

The overview of each task is as follows:

* T1: Obstacle Traversal

 A task to navigate through a tunnel made uneven by scattered debris and conduct internal investigations. This tests the robot's mobility. Visibility may be impaired due to smoke from fires.

* T2: Vehicle Investigation

 A task to investigate vehicles stranded in the tunnel. This involves checking whether victims are trapped inside vehicles and looking for any abnormalities. The investigation tests both operational capabilities for opening vehicle doors and visual recognition abilities for internal inspection.

* T3: Tool-assisted Vehicle Investigation and Rescue

 A task to rescue victims from accident vehicles where victims are trapped. Using a hydraulic spreader to pry open doors that won't open due to the accident, then extracting victims from inside the vehicle. Victims must be handled carefully to avoid injury. This tests more advanced operational capabilities than Task T2.

* T4: Route Clearing

 A task to remove obstacles scattered in the tunnel to secure a path for other vehicles and people to enter. This tests the robot's operational capabilities and motion planning abilities.

* T5: Firefighting

 A task to conduct firefighting activities using fire hydrants against fires occurring in the tunnel. This requires completing a series of operations: opening the fire hydrant door, connecting the water nozzle to the hose end, opening the valve, extending the hose, turning on the water lever, and manipulating the hose end to direct water at the fire source. This tests advanced operational capabilities.

* T6: Shoring and Breaching

 A task to investigate inside a vehicle trapped under a collapsed wall. First, stabilize the collapsed wall around the investigation area by inserting support tools to prevent unwanted movement—this operation is called shoring. Next, drill holes in the shored wall—this operation is called breaching. Then, investigate inside the trapped vehicle by inserting an arm through the holes. This task also tests advanced operational capabilities.

In the competition, scoring points are set for each task, and points are accumulated by clearing these objectives. Each task has a time limit, and the goal is to score as many points as possible and complete the task within that time. When a task is completed, the time taken is also factored into the score (faster completion yields higher scores). The final competition ranking is determined by the total score across all six tasks.

Note that up to two robots can be deployed for task execution and can work cooperatively. Effective use of robot cooperation may make it easier to achieve higher scores.

For details on tasks and scoring points, please refer to the `rulebook <http://worldrobotsummit.org/download/rulebook-en/rulebook-Tunnel_Disaster_Response_and_Recovery_Challenge.pdf>`_.

.. _wrs2018_overview_operation:

Robot Operation
---------------

On the simulation PC side, participants can install and execute their complete set of robot control software. The control software is basically implemented as a Choreonoid controller item. However, you may also build a control software system external to the controller item and connect that system to the controller item. In that case, you can use middleware such as ROS or OpenRTM. In any case, regarding robot operation, the simulation PC corresponds to the control PC mounted on the actual robot.

The environment used for robot control and the robot's state must be obtained only from sensors mounted on the robot (cameras, LiDAR sensors, force sensors, acceleration sensors, rate gyros, etc.). This means performing control under the same conditions as the actual robot. While the simulator, unlike actual robots, can obtain views from arbitrary viewpoints or global coordinate values of the robot, such information cannot be used for control.

If the robot control software built on the simulation PC operates completely autonomously, robot operation can be completed with this alone. However, since this competition includes advanced tasks, fully automating robot operation may be challenging.

Therefore, for robot operation, it's also possible to prepare a separate operation PC for remote control. The PC used for this is prepared and brought by each participating team. The simulation PC and remote operation PC are network-connected and communicate via TCP/IP. Of course, you may use ROS or OpenRTM based on TCP/IP for this communication. As long as it's based on TCP/IP, you may use other communication systems or proprietary communication systems. However, note that the communication target on the simulation PC side must only be the robot control system. Direct access to the simulator from the remote operation PC to obtain information not normally available from the robot is prohibited.

The above configuration is illustrated as follows:

.. image:: images/teleop-overview.png

Note that multiple remote operation PCs may be used. However, the power capacity available for operation PCs is limited to 1500W, so usage must be within this range. Also, the size of tables for installing operation PCs at the venue is fixed, and equipment must fit within these constraints.

Additionally, communication failures may occur between the simulation PC and remote operation PCs, such as communication delays or packet loss. These can also occur at actual disaster sites. While communication failures are occurring, remote operation will also be affected. In this case, teams with more autonomous robot operations should be able to proceed with tasks more efficiently. The occurrence and frequency of communication failures in actual competitions will be adjusted by the competition organizers considering the difficulty level.

About Competition Simulation Samples
------------------------------------

We have prepared samples for conducting this competition's simulations on Choreonoid. The following sections explain how to run these samples and their contents, so please try these samples first. This should give you an idea of the competition overview and the preparations needed for participation. After that, you can modify the samples to accommodate your own robot models, control software, and remote operation systems.