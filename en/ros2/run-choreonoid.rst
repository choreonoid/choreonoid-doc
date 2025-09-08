Running Choreonoid in ROS 2 Environment
=======================================

.. contents::
   :local:

.. highlight:: sh

.. _choreonoid_ros2_run_choreonoid:

Starting Choreonoid
-------------------

In the ROS 2 environment, you can start Choreonoid with the following command: ::

   ros2 run choreonoid_ros choreonoid

Here, we use the "ros2 run" command to launch "choreonoid", which is the executable file of the "choreonoid_ros" package.

If the startup is successful, the Choreonoid main window will be displayed. This is basically the same as when Choreonoid is started in the normal way, and the operation method is also the same.

You can also add various options for Choreonoid after this command.

.. note:: Choreonoid main body's normal startup command "choreonoid" cannot use ROS 2 integration features. When using ROS 2 integration features, please start Choreonoid using the above method.