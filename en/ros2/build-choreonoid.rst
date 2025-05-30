Building Choreonoid-related Packages
====================================

Here we will build (install) Choreonoid as one of the ROS 2 packages. We will also build several Choreonoid-related packages.

This document installs Choreonoid using a different procedure from :doc:`../install/build-ubuntu`. Even if you have already installed Choreonoid using that procedure, please note that you will be installing a separate Choreonoid for ROS 2 independently.

We use colcon to build Choreonoid and its related packages, just like other ROS packages.
For a more detailed explanation of colcon, please refer to the following pages in the official ROS 2 documentation:

* `ROS 2 Documentation: Jazzy - Using colcon to build packages <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_
* `ROS 2 Documentation: Humble - Using colcon to build packages <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_


.. contents::
   :local:

.. highlight:: sh


Creating a ROS 2 Workspace
--------------------------

Create a ROS 2 workspace for Choreonoid.

The workspace is usually created in your home directory. Here, we'll name the workspace "ros2_ws". This name can be set freely. If you use a different workspace name, replace "ros2_ws" in the following explanations with your chosen name.

First, create an empty workspace: ::

   mkdir -p ~/ros2_ws/src
   cd ros2_ws

.. _add_choreonoid_package_sources_for_ros2:

Adding Package Sources
----------------------

Clone the source code repositories for Choreonoid main body and ROS 2 plugin into the "src" directory of the created workspace: ::

   cd src
   git clone https://github.com/choreonoid/choreonoid.git
   git clone https://github.com/choreonoid/choreonoid_ros.git

These correspond to the following GitHub repositories:

* `choreonoid <https://github.com/choreonoid/choreonoid>`_ : Choreonoid main body
* `choreonoid_ros <https://github.com/choreonoid/choreonoid_ros>`_ : ROS package for using ROS 1/2 functionality with Choreonoid

.. note:: Choreonoid's ROS 2 integration features target Choreonoid version 2.1.1 or later. The above repository cloning uses the latest development version.

Please keep the contents of each repository as up-to-date as possible.


Installing Dependencies for Choreonoid Main Body
------------------------------------------------

Install the dependency packages required for building and running Choreonoid main body.

A script for installing dependency packages for Ubuntu is already prepared, so we'll use that.
Please note that this method differs from the usual dependency package installation method in ROS.

Move to the Choreonoid source directory: ::

  cd choreonoid

Execute the corresponding script. For Ubuntu 24.04: ::

   misc/script/install-requisites-ubuntu-24.04.sh

For Ubuntu 22.04, execute: ::

   misc/script/install-requisites-ubuntu-22.04.sh

Note that if you have already installed the latest Choreonoid on your OS independently of ROS 2, you don't need to perform this task again.

.. _install-choreonoid-ros2-dependencies:

Installing Dependencies for choreonoid_ros
------------------------------------------

The choreonoid_ros package depends on several ROS packages, which also need to be installed. Install these dependency packages using the standard ROS method. Specifically, execute the rosdep command as follows: ::

   rosdep install -y --from-paths ~/ros2_ws/src --ignore-src

This command will additionally install the dependency packages described in the "package.xml" of choreonoid_ros.


.. _ros2_colcon_build_command:

Building
--------

Let's build using the following command. The directory where you execute the command must be the top of the workspace: ::

   cd ~/ros2_ws
   colcon build --symlink-install

The build option `--symlink-install` installs various files using symbolic links during installation. This consumes less PC storage space since file copying doesn't occur, and for files that don't require compilation, edited content is reflected immediately. For example, in Choreonoid, .body files and .project files, and in ROS 2, .urdf files and .yaml files are subject to immediate reflection of edits.

For details on this command's options, please refer to `build - Build Packages <https://colcon.readthedocs.io/en/released/reference/verb/build.html>`_ in the `official colcon documentation <https://colcon.readthedocs.io/en/released/index.html>`_.

If the build is successful, messages like the following will be output:

.. code-block:: none

   Starting >>> choreonoid
   Finished <<< choreonoid
   Starting >>> choreonoid_ros
   Finished <<< choreonoid_ros

   Summary: 2 packages finished

Note that CMake option settings are possible with the colcon command. For details, see :ref:`ros2_build_choreonoid_cmake_options`.

.. _loading_ros2_workspace_setup_script:

Loading the Workspace Setup Script
----------------------------------

After building, a file called "setup.bash" is generated in the workspace's install directory. The settings described in this script are necessary when executing packages in the workspace, so set it to be executed by default. Usually, add the following to the .bashrc file in your home directory: ::

   source $HOME/ros2_ws/install/setup.bash

This way, the file will be executed automatically when you start a terminal, and the settings will be loaded.

You can add the above command with the following command: ::

   echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc

Since this setting hasn't been loaded yet during the initial build, restart the terminal or directly input the above source command from the command line to apply the settings.

.. note:: This script is **different** from the ROS 2 main body setup.bash introduced in :doc:`install-ros2`, so please be careful. To make packages on the workspace work properly, you need to load both scripts.

Reference: Using the Package Management Tool
--------------------------------------------

In ROS 2, `vcstool <https://github.com/dirk-thomas/vcstool>`_ is available as a standard tool for managing multiple packages together. Using this, you can clone and update multiple repositories in bulk.

You can install vcstool with the following command (it's installed with :ref:`ros2_install_ros2_install_dev_tools`): ::

   sudo apt install python3-vcstool


Check the usage with: ::

   vcs help

By executing: ::

 vcs pull

in a directory above each repository, git pull is executed for all repositories, allowing you to update all repositories to the latest version.

For example, with the following commands, you can update all clones in the "src" directory, including choreonoid and choreonoid_ros introduced in :ref:`add_choreonoid_package_sources_for_ros2`, to the latest version: ::

   cd ~/ros2_ws
   vcs pull src


.. _ros2_build_choreonoid_cmake_options:

Reference: Setting CMake Options
--------------------------------

If you want to set CMake options when building Choreonoid, use the "--cmake-args" option of the colcon command.

For example, you can set an option to disable the generation of Choreonoid's regular executable file. During ROS 2 integration, the choreonoid_ros package generates Choreonoid's executable file. Therefore, there will be both Choreonoid's regular executable file and the ROS 2 executable file. By building with the "BUILD_CHOREONOID_EXECUTABLE" option set to OFF as follows instead of the build command introduced in :ref:`ros2_colcon_build_command`, the former regular executable file will not be generated: ::

   colcon build --symlink-install --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF


It's also possible to enable Choreonoid's optional plugins using CMake options. For example, if you want to use the "Media Plugin" for playing video and audio files on Choreonoid, do the following: ::

   colcon build --symlink-install --cmake-args -DBUILD_MEDIA_PLUGIN=ON

If you want to set multiple options, just list the options. For example, the following command sets both disabling regular executable generation and building the media plugin: ::

   colcon build --symlink-install --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DBUILD_MEDIA_PLUGIN=ON

.. note:: With this setting method, these options will be enabled for all packages in the workspace, so be careful that unintended options may be enabled for other packages.

.. note:: The BUILD_MEDIA_PLUGIN option mentioned above is just an example for explanation and is not necessarily required when using Choreonoid with ROS. If you don't need to play media files such as videos on Choreonoid, you don't need to turn this option ON.


In this way, you can set CMake options even in the ROS 2 environment. If there are options you want to use in the ROS 2 environment, enable them accordingly.

.. _ros2_catkin_config_cmake_build_type:

Reference: Setting Build Type
-----------------------------

Generally, when building C/C++ programs, you can specify build types such as "Release" or "Debug". In Release mode, optimization is applied to increase execution speed, while in Debug mode, debug information is added to facilitate debugging with a debugger.

If you want to specify these build types when building with the colcon command, use the --cmake-args option again.

For example: ::

   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

will build in release mode, and: ::

   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

will build in debug mode.

These should be specified in addition to the options specified in :ref:`ros2_build_choreonoid_cmake_options`.

Choreonoid-related packages are set to Release by default. However, in general, some packages may not set the build type to Release by default, and your own packages may not have this setting. In that case, optimization will not be applied, and the execution speed of the built program will be significantly reduced. If you might build such packages, it's good to specify Release build using the above method.

Reference: Using Multiple Choreonoid Environments Together
----------------------------------------------------------

Here we introduced how to install Choreonoid that operates in a ROS 2 environment (ROS 2 workspace). As mentioned at the beginning, Choreonoid can also be installed independently of ROS 2. However, if you use them together on the same OS, some caution is required.

When the ROS 2 environment setup script is loaded into the system, the corresponding directory of the ROS 2 workspace is added to the shared library path. (It's added to the environment variable LD_LIBRARY_PATH.) In this state, if there are multiple shared libraries with the same name on the system, the ROS 2 environment ones will usually be loaded preferentially. If this is applied to software originally installed independently of ROS 2, libraries with different versions or build settings may be loaded, causing the software to malfunction. Mixing multiple environments is dangerous.

To avoid this, it's safer to disable the loading of the "setup.bash" scripts mentioned in :ref:`loading_ros2_workspace_setup_script` and :doc:`install-ros2` when using software independent of ROS 2. You can disable them by commenting out the relevant parts in the ".bashrc" configuration file and then restarting the OS or terminal.

Note that for Choreonoid, it's possible to execute without mixing with libraries built in other environments through RPATH information embedded in executable files and shared library files. This feature is enabled by default for executable files and libraries generated in the build directory. Also, by turning ON CMAKE's ENABLE_INSTALL_RPATH, this is also enabled for files installed by "make install".

Through such mechanisms, Choreonoid's shared libraries are designed to avoid mixing with those from other environments as much as possible. However, depending on the environment settings, they may still mix, and libraries may mix in other software used in conjunction with Choreonoid. Therefore, as a general matter not limited to Choreonoid, when the same software is installed in multiple environments on the same OS, it's very important to use them without mixing to avoid problems.

.. note:: If you turn ON the "ENABLE_NEW_DTAGS" option in CMake options when building Choreonoid, LD_LIBRARY_PATH information will be prioritized over RPATH, increasing the risk of mixing. If this option is not particularly necessary, please leave it at the default OFF.