Preparation
===========

.. contents::
   :local:

.. _ros_tank_tutorial_package_setup:


Setup of ROS and Choreonoid related packages
---------------------------------------------

In order to follow this tutorial, it is necessary to do :doc:`../install-ros` and  :doc:`../build-choreonoid` first.
Follow the steps in these pages for installation and building so that you can do :doc:`../run-choreonoid` .

When adding package sources, it is OK if the following three Choreonoid-related packages are installed.

* choreonoid
* choreonoid_ros
* choreonoid_joy

In the above page, choreonoid_ros_samples is also installed, but it is not necessary for this tutorial. 
(Of course thare is no problem even if it is installed.)

.. _ros_tank_tutorial_make_package:


Creating a package for the tutorial
-----------------------------------

.. highlight:: sh

Basically, data and programs to be created/developed and used on ROS are built in Catkin "packages".
Therefore, in this tutorial, let's first create a package to store the programs and data to be created in the tutorial.

A package can be created by executing the following command on the catkin workspace. ::

 catkin create pkg choreonoid_ros_tank_tutorial

This will create a package named "choreonoid_ros_tank_tutorial". The corresponding directory will be created in the "src" directory in the workspace, and some files related to the package will be added to it.

In this tutorial, we will use the package name "choreonoid_ros_tank_tutorial". It is a little long, but please understand that this is to make it consistent with other Choreonoid related packages. You may use any other name, but in that case, please replace all the corresponding parts in the tutorial.

The source of the package for this tutorial is available on Github. By cloning the repository, you can try this tutorial without typing the source code. In this case, instead of the above command, execute the following. ::

 cd ~/catkin_ws/src
 git clone https://github.com/choreonoid/choreonoid_ros_tank_tutorial.git

You will now be in the same situation as if you had created the package and created all the files required for this tutorial.

.. _ros_tank_tutorial_edit_package_xml:


Editing package.xml
-------------------

.. highlight:: xml

With the above operation, an XML file called "package.xml" has been added to the package directory. This file is used to describe the package information and is required for every package.

The contents of package.xml can be configured by the command line options when executing the package creation command. Please refer to the `catkin create manual <https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html>`_ for details.

The contents of package.xml can also be edited directly using a text editor. Therefore, it is possible to generate the package without any options and then manually modify the package.xml to make it appropriate. It may be easier to edit the package.xml manually, as the command line options can be complicated. In this tutorial, we will not discuss the options of the package generation command, but will explain the contents of package.xml that should be created for this tutorial. The actual contents of package.xml should look like this ::

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

Please refer to the `package.xml manual <http://wiki.ros.org/catkin/package.xml>`_ for the details of the description.
We will explain some important parts here.

First, the following tag starts the package description and specifies that the format of the description is version 2. ::

 <package format="2">.

There are two implementations of Catkin, an old one and a new one, each with slightly different usage (see :ref:`ros_make_catkin_workspace`_ ). In this tutorial, we will use the new implementation, and this description is provided to accommodate it.

Next, the following description specifies the package name. ::

 <name>choreonoid_ros_tank_tutorial</name>

This must not overlap with any other package.

The other important part is the following part. ::

   <buildtool_depend>catkin</buildtool_depend>
   <depend>choreonoid</depend>
   <depend>choreonoid_ros</depend>
   <depend>choreonoid_joy</depend>
   <depend>std_msgs</depend>
   <depend>sensor_msgs</depend>
   <depend>image_transport</depend>

This specifies the other packages that this package depends on.
Here we describe the dependencies on the following packages.

* choreonoid: Choreonoid itself
* choreonoid_ros: ROS integration function of Choreonoid
* choreonoid_joy: Joystick node for Choreonoid 
* std_msgs: Standard message types for ROS
* sensor_msgs: Message types corresponding to standard sensors
* image_transport: A function for image data communication.

Of course, the packages for Choreonoid and the ROS integration function are required, and the other packages will be explained in this tutorial as needed.

Finally, the folowing describtion is given. ::

   <export>
     <build_type>cmake</build_type>
   </export>

This is actually not very common in ROS. This "build_type" is an option for how to describe the package build with CMake. There are two options, each described in the `supported build types section of catkin tools <https://catkin-tools.readthedocs.io/en/latest/build_types.html>`_  as follows:

* **catkin**: CMake packages that use the Catkin CMake macros
* **cmake**: "Plain" CMake packages

The default type is "catkin", which uses the CMake macros customized by Catkin to build the package.
On the other hand, if you set it to "cmake", it will not use such macros, but will use the normal CMake writing method.

Normally, the default is fine, but it seems to assume that the built files are placed in the filesystem locations determined by Catkin. For example, there are specific locations for node executables, library files, and so on.

On the other hand, there may be cases where you want to place the built files in other locations. For example, Choreonoid has the dedicated directories for plugins and controller binaries written in C++, and they are usually stored there. However, as far as the author tried, it is not clear how to do this with the default "catkin" build option. 
So the author tried the "cmake" build option and found out that it is possible to do so.
Since this tutorial will be developing controllers, the "cmake" build option will be used.

By changing this option, the description of the CMake file for building the package may be slightly different.
However, since these parts are basically described in the CMake macros of Choreonoid itself and choreonoid_ros package,
the users of those packages do not need to worry about it too much. 

.. note:: The above explanation is based on the author's own trial and error, and there is no proof that this is the right way.
While ROS is easy to use when used in the prescribed way, when you try to do something a little different from the prescribed way, there is not much information or it may be difficult to realize. There doesn't seem to be much explanation about this issue in the manual or discussion in the forums either. If you know anything about this, it would be great if you could let me know.

File structure of the package
-----------------------------

The source of the package will have the following file structure after this step.

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   - CMakeLists.txt
   - package.xml

Starting the ROS master
-----------------------

.. highlight:: sh

Open a terminal and type the following command to start the ROS master. ::

  roscore

If you use roslaunch command, it will be started automatically if ROS master is not present. Since we will be using roslaunch in the tutorial, it may not be necessary to explicitly start the ROS master, but in general, you should do this beforehand.

Start the working terminal
--------------------------

Apart from launching the ROS master, open a terminal for working on the tutorial, and move it to the tutorial directory above. Note that you may need more than one terminal to proceed with the tutorial.
