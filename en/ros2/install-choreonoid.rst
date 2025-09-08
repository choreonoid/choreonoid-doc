Installing Choreonoid Main Body
===============================

This section explains how to install Choreonoid that integrates with ROS 2.

.. contents::
   :local:

.. highlight:: sh

Choreonoid Main Body Installation Methods
------------------------------------------

For installing Choreonoid main body that integrates with ROS 2, you have the following options:

1. Install independently of ROS 2, using the standard installation method
  A. Install from packages
  B. Build and install from source code
2. Install as a ROS 2 package

.. _ros2_install_choreonoid_standard_method:

Standard Installation Method
----------------------------

If you already have Choreonoid main body installed, using it is simple, reliable, and recommended.
However, please note that this method became available from Choreonoid main body version 2.3.0 onwards.

The simplest installation method for Choreonoid main body is option 1-A above: :doc:`package installation <../install/install-package-ubuntu>`.

If necessary, you may also perform option 1-B: :doc:`build and install from source code <../install/build-ubuntu>`.
In that case, make sure to also perform the :ref:`post-build installation work <build-ubuntu_install>`.
Since the installation path needs to be accessible, ensure that the setup.bash script is also executed reliably.

.. _ros2_install_choreonoid_register_to_rosdep:

Registering Choreonoid Installed via Standard Method with rosdep
-----------------------------------------------------------------

When you install Choreonoid main body using the standard installation method, registering it with rosdep so that rosdep can recognize Choreonoid will enable proper resolution of dependencies from other ROS 2 packages.

Register with rosdep using the following procedure.

First, create a rosdep source list file: ::

 echo "yaml file:///etc/ros/rosdep/choreonoid.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-choreonoid.list

Next, create a rule definition file for Choreonoid.

For package installation: ::

 sudo tee /etc/ros/rosdep/choreonoid.yaml << EOF
 choreonoid:
   ubuntu: choreonoid
 EOF

For installation built from source code: ::

 sudo tee /etc/ros/rosdep/choreonoid.yaml << EOF
 choreonoid:
   ubuntu:
     source:
       uri: 'file:///usr/local'
 EOF

Change the ``/usr/local`` part above to match your actual Choreonoid installation path.

After creating the configuration files, update the rosdep database: ::

 rosdep update

Once the update is complete, you can verify with the following command: ::

 rosdep resolve choreonoid

This enables rosdep to recognize Choreonoid properly even when other ROS 2 packages specify Choreonoid as a dependency package in their ``package.xml``.

.. note:: By performing this configuration, when you run the ``rosdep install`` command, it will recognize that Choreonoid is already installed, allowing you to avoid unnecessary reinstallation or errors.

.. _ros2_install_choreonoid_install_as_ros2_package:

Installing as a ROS 2 Package
------------------------------

This method was used previously.
As explained in the next :doc:`build-choreonoid`, Choreonoid main body is also built from source in the ROS 2 workspace.

If you don't have a particular need to use this method, please use the "Standard Installation Method".

Note that when installing with this method, since Choreonoid exists as a package within the ROS 2 workspace, registration with rosdep is not necessary.

Also, when using this method, if you already have Choreonoid installed, it may interfere with the existing installation.
Be careful with configuration to avoid interference in paths for executable files, shared libraries, CMake, include files, etc.
For details, please refer to :ref:`ros2_build_choreonoid_as_ros2_package_note`.