ROS2 Plugin
===========

.. contents::
   :local:

.. highlight:: sh

What is the ROS2 Plugin?
------------------------

The ROS2 plugin is a plugin for Choreonoid included in the choreonoid_ros package.

This plugin provides information about simulations on Choreonoid via ROS 2 topics.

Currently, the WorldROS2 item that provides simulation time via the `/clock` topic and the BodyROS2 item that provides robot sensor data are available.


Loading the ROS2 Plugin
-----------------------

The ROS2 plugin is included in the choreonoid_ros package. To use the ROS2 plugin, you need to start Choreonoid via choreonoid_ros. As mentioned in :ref:`choreonoid_ros2_run_choreonoid`, you can start it using the ros2 run command: ::

   ros2 run choreonoid_ros choreonoid

Alternatively, you can start it using the ros2 launch command: ::

   ros2 launch choreonoid_ros choreonoid.launch.xml


In this way, when Choreonoid is started as part of the choreonoid_ros package using the ros2 run/launch command, the ROS2 plugin is automatically loaded. At that time, the following message will be output in the message view of the started Choreonoid: ::

   ROS2-plugin has been activated

If this message is not output, the ROS2 plugin has not been loaded. In this case, please note that the ROS 2 integration features on Choreonoid will not be available. (This will be the case if you start Choreonoid by methods other than those described above, or if you start Choreonoid installed independently of ROS in the normal way.)