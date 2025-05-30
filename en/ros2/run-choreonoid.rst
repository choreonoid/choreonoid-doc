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

.. note:: The "choreonoid" package corresponding to Choreonoid main body also includes an executable file "choreonoid". You can also execute it by simply typing "choreonoid". However, in that case, the :doc:`rosplugin` will not be loaded, so the ROS 2 integration features cannot be used. When using ROS 2 integration features, please start Choreonoid using the above method. To avoid confusion, it's recommended to not build the "choreonoid" command of Choreonoid main body by using the "--cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF" option introduced in :ref:`ros2_build_choreonoid_cmake_options`.