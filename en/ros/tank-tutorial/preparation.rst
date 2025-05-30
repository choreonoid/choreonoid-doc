Preparation
===========

.. contents::
   :local:

.. _ros_tank_tutorial_package_setup:

Setting Up ROS and Choreonoid-related Packages
-----------------------------------------------

To proceed with this tutorial, you must first complete :doc:`../install-ros` and :doc:`../build-choreonoid`. Follow the procedures on those pages to install and build the necessary components, ensuring you can perform :doc:`../run-choreonoid`.

Regarding :ref:`ros_choreonoid_add_package_sources`, you need to have the following three Choreonoid-related packages installed:

* choreonoid
* choreonoid_ros
* choreonoid_joy

While the referenced page also introduces choreonoid_ros_samples, it's not required for this tutorial. (Of course, having it installed won't cause any issues.)

.. _ros_tank_tutorial_make_package:

Creating a Tutorial Package
---------------------------

.. highlight:: sh

Data and programs newly created and developed for use on ROS are fundamentally built in units called Catkin "packages". Therefore, in this tutorial, we'll first create a package to store the programs and data we'll be creating.

Create a package by executing the following command in your catkin workspace: ::

 catkin create pkg choreonoid_ros_tank_tutorial

This creates a package named "choreonoid_ros_tank_tutorial". A corresponding directory is generated in the workspace's "src" directory, containing several package-related files.

This tutorial uses the package name "choreonoid_ros_tank_tutorial". While it's somewhat long, this maintains consistency with other Choreonoid-related packages. You may use a different name if you prefer, but in that case, be sure to replace all corresponding references throughout the tutorial.

Note that the source code for this tutorial package is publicly available on GitHub. By cloning this repository, you can try the tutorial without manually entering the source code. To do this, execute the following instead of the above command: ::

 cd ~/catkin_ws/src
 git clone https://github.com/choreonoid/choreonoid_ros_tank_tutorial.git

This puts you in the same state as if you had created the package and all necessary files for this tutorial.

.. _ros_tank_tutorial_edit_package_xml:

Editing package.xml
-------------------

.. highlight:: xml

The above operation adds an XML file called "package.xml" to the package directory. This file describes package information and is required for all packages.

You can configure the contents of package.xml through command-line options when executing the package creation command. For details, refer to the `catkin create manual <https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html>`_.

Alternatively, you can directly edit package.xml using a text editor. This allows you to first generate the package without options, then manually modify package.xml as needed. Since command-line option specification can become complex, manual editing might be more straightforward. Here, without delving into package generation command options, we'll explain the package.xml content needed for this tutorial. The actual package.xml should contain the following: ::

 <?xml version="1.0"?>
 <package format="2">
   <name>choreonoid_ros_tank_tutorial</name>
   <version>1.0.0</version>
   <description>
     Tutorial on how to implement a robot controller with I/O using ROS
   </description>
   <maintainer email="nakaoka@choreonoid.co.jp">Shin'ichiro Nakaoka</maintainer>
   <license>MIT</license>
   <author email="nakaoka@choreonoid.co.jp">Shin'ichiro Nakaoka</author>
   <url type="website">http://choreonoid.org</url>
   <url type="repository">https://github.com/choreonoid/choreonoid_ros_tank_tutorial.git</url>
   <buildtool_depend>catkin</buildtool_depend>
   <depend>choreonoid</depend>
   <depend>choreonoid_ros</depend>
   <depend>choreonoid_joy</depend>
   <depend>std_msgs</depend>
   <depend>sensor_msgs</depend>
   <depend>image_transport</depend>
   <export>
     <build_type>cmake</build_type>
   </export>
 </package>

For detailed descriptions, please refer to the `package.xml manual <http://wiki.ros.org/catkin/package.xml>`_. Here we'll explain some important elements.

First, the tag: ::

 <package format="2">

starts the package description and explicitly states that the description format is version 2.

Catkin has both old and new implementations with slightly different usage (see :ref:`ros_make_catkin_workspace`). This tutorial uses the new implementation, and this notation accommodates that.

Next: ::

  <name>choreonoid_ros_tank_tutorial</name>

specifies the package name. This must be unique and not conflict with other packages.

Also important is: ::

  <buildtool_depend>catkin</buildtool_depend>
  <depend>choreonoid</depend>
  <depend>choreonoid_ros</depend>
  <depend>choreonoid_joy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>image_transport</depend>

This section explicitly declares dependencies on other packages:

* choreonoid: Choreonoid core
* choreonoid_ros: Choreonoid's ROS integration features
* choreonoid_joy: Joystick node for Choreonoid
* std_msgs: ROS standard message types
* sensor_msgs: Message types for standard sensors
* image_transport: Features for image data communication

Each package except Choreonoid core and ROS integration features will be explained as needed throughout this tutorial.

Finally: ::

  <export>
    <build_type>cmake</build_type>
  </export>

This is actually not a standard notation in ROS. The "build_type" is an option related to how CMake describes package building. There are two choices, explained in `Catkin tools' Supported Build Types <https://catkin-tools.readthedocs.io/en/latest/build_types.html>`_ as follows:

* **catkin**: CMake packages that use the Catkin CMake macros
* **cmake**: "Plain" CMake packages

The default is "catkin", which builds using CMake macros customized by Catkin.
Setting it to "cmake" uses standard CMake notation without those macros.

The former assumes built files are placed in Catkin-determined locations.
For example, specific locations are designated for node executables and library files.

However, you might want to place built files elsewhere. For instance, Choreonoid has dedicated directories for C++ plugins and controller binaries, where they're typically stored. However, based on the author's testing, it's unclear how to achieve this with the default "catkin" build option.
Testing the "cmake" build option revealed this was possible. Since this tutorial involves controller development, we'll use the "cmake" build option.

Changing this option may slightly alter the CMake file descriptions for building packages.
However, since these aspects are primarily described in CMake macros within Choreonoid core and the choreonoid_ros package, users of these packages shouldn't need to worry much about this.

.. note:: The above explanation is based on the author's trial and error, and there's no certainty this is the best approach. While ROS is convenient when used as prescribed, attempting something slightly outside the standard can be challenging due to limited information or implementation difficulties. There seems to be little manual explanation or forum discussion on this particular issue. If you have any knowledge about this topic, your input would be greatly appreciated.

Package File Structure
----------------------

At this point, the package source has the following file structure:

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   - CMakeLists.txt
   - package.xml

Starting the ROS Master
-----------------------

.. highlight:: sh

Open a terminal and enter the following to start the ROS master: ::

  roscore

When using the roslaunch command, the ROS master starts automatically if not already running. Since we'll use roslaunch in this tutorial, explicitly starting the ROS master may not always be necessary, but it's generally good practice to do this beforehand.

Launching a Work Terminal
-------------------------

Separate from the ROS master terminal, open a terminal for tutorial work and navigate to the tutorial directory mentioned above. Note that you may need multiple terminals as you progress through the tutorial.