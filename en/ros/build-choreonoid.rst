Building Choreonoid-related Packages
====================================

Here we will build (install) Choreonoid as a package in the ROS environment. We will also build several Choreonoid-related packages.

This document describes a different installation procedure from :doc:`../install/build-ubuntu`. Please note that even if you have already installed Choreonoid using that procedure, you will need to install a separate version of Choreonoid for ROS independently. Please manage the Choreonoid installed here separately from the one installed through the standard procedure.

.. It is possible to use Choreonoid installed through the standard procedure in the ROS environment, but the necessary ROS packages and documentation for that are currently being prepared. For now, please follow the instructions in this document to install Choreonoid for ROS.

.. contents::
   :local:

.. highlight:: sh

.. _ros_make_catkin_workspace:

Creating a Catkin Workspace
---------------------------

Create a Catkin workspace for Choreonoid.

The workspace is typically created in your home directory. The workspace is usually named "catkin_ws". You can change this name if you wish, but in that case, please replace "catkin_ws" in the following instructions with your chosen name.

First, create an empty workspace: ::

 mkdir catkin_ws
 cd catkin_ws
 mkdir src
 catkin init

.. note:: Here we use the "catkin init" command to initialize the workspace, but there is also a command called "catkin_init_workspace" that performs a similar operation. The former is a command included in `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_, which is the newer command system for Catkin, while the latter is from the older format of Catkin. These two systems use different commands for building packages as well: "catkin build" and "catkin_make" respectively. Since Choreonoid's ROS integration assumes the use of Catkin Command Line Tools, please do not use the older format Catkin commands (such as catkin_make).

.. _ros_choreonoid_add_package_sources:

Adding Package Sources
----------------------

Clone the source code repositories for Choreonoid itself and the ROS plugin into the "src" directory of the created workspace: ::

 cd src
 git clone https://github.com/choreonoid/choreonoid.git
 git clone https://github.com/choreonoid/choreonoid_ros.git

These correspond to the following GitHub repositories:

* `choreonoid <https://github.com/choreonoid/choreonoid>`_ : Choreonoid core
* `choreonoid_ros <https://github.com/choreonoid/choreonoid_ros>`_ : ROS package for using ROS functionality in Choreonoid

.. note:: Choreonoid's ROS integration functionality currently targets the development version of Choreonoid. By cloning the above repositories, you can use the latest development version.

If you plan to follow the explanations in the subsequent sections, you should also clone the samples used there: ::

 git clone https://github.com/choreonoid/choreonoid_ros_samples.git
 git clone https://github.com/choreonoid/choreonoid_joy.git

These correspond to the following GitHub repositories:

* `choreonoid_ros_samples <https://github.com/choreonoid/choreonoid_ros_samples>`_ : Samples for using ROS with Choreonoid
* `choreonoid_joy <https://github.com/choreonoid/choreonoid_joy>`_ : ROS node for using a joystick (gamepad) with Choreonoid's mapping

Please keep the contents of each repository as up-to-date as possible.

.. note:: Other Choreonoid ROS-related packages include `choreonoid_ros_pkg <https://github.com/fkanehiro/choreonoid_ros_pkg>`_ and `choreonoid_rosplugin <https://github.com/s-nakaoka/choreonoid_rosplugin>`_. These are not currently officially supported, so please do not introduce them when using Choreonoid with this manual's method. If these are installed as binary packages or included in your catkin workspace, the features explained in this document may not work properly. When following this manual, please create a new workspace and first introduce only the specified packages to verify operation.


Using Repository Management Tools
---------------------------------

Tools for managing multiple repositories together include `wstool <http://wiki.ros.org/wstool>`_ and `vcstool <https://github.com/dirk-thomas/vcstool>`_. These tools allow you to update multiple repositories at once, which can be very useful.

Here's a brief introduction to vcstool. You can install vcstool with: ::

 sudo apt install python3-vcstool

Check the usage with: ::

 vcs help

In a directory above each repository, executing: ::

 vcs pull

will run git pull on all repositories, updating them all to the latest version.

Installing Dependencies
-----------------------

Install the dependency packages required for building and running Choreonoid.

Navigate to Choreonoid's source directory and run the corresponding script. For Ubuntu 20.04: ::

 misc/script/install-requisites-ubuntu-20.04.sh

This process should ideally be handled by Catkin's dependency package information, but since this is not yet complete for Choreonoid, this step is necessary to ensure proper installation.

Note that if you have already installed the latest Choreonoid on your OS independently of ROS, this step should already be completed and doesn't need to be repeated.

.. _ros_build_choreonoid_cmake_options:

Setting CMake Options
---------------------

To set CMake options when building Choreonoid, use the "--cmake-args" option of the catkin config command.

First, it's recommended to disable the generation of Choreonoid's regular executable file by running: ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF

In ROS, programs are launched as "nodes" instead of regular executables, and the node executable for Choreonoid is included in the choreonoid_ros package. Having both the regular executable and the node version can be confusing, but the above option helps avoid this.

You can also enable optional Choreonoid plugins. For example, to use the "Media Plugin" for playing video and audio files in Choreonoid: ::

 catkin config --cmake-args -DBUILD_MEDIA_PLUGIN=ON

To set multiple options, simply list them. For example, the following command sets both the prohibition of regular executable generation and the Media Plugin build: ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DBUILD_MEDIA_PLUGIN=ON

After configuration, run: ::

 catkin config

to display the workspace settings. If you see something like:

.. code-block:: none

 Additional CMake Args: -DBUILD_CHOREONOID_EXECUTABLE=OFF -DBUILD_MEDIA_PLUGIN=ON

then it's correctly configured.

.. note:: Setting options this way will apply them to all packages in the workspace, which might enable unintended options in other packages. However, Catkin doesn't have a feature to set CMake options individually for each package (`there was a request but it was declined <https://github.com/catkin/catkin_tools/issues/205>`_), so we have to use this approach.

.. note:: The BUILD_MEDIA_PLUGIN option mentioned above is just an example for explanation and is not necessarily required when using Choreonoid with ROS. If you don't need to play media files like videos in Choreonoid, you don't need to enable this option.

To clear the set options: ::

 catkin config --no-cmake-args

Please enable any options you want to use in the ROS environment using the method described above.

.. _note_on_ros_python_version:

Note on Python Version
^^^^^^^^^^^^^^^^^^^^^^

Choreonoid builds the Python plugin and Python wrapper library by default, and the Python used there is Python3 by default.
This is the same as the Python used in ROS Noetic Ninjemys for Ubuntu 20.04, so the default Choreonoid settings should work fine normally.
However, please note that if you set Choreonoid to use Python2 through option settings, it will not work properly.

.. _ros_catkin_config_cmake_build_type:

Setting the Build Type
----------------------

Generally, when building C/C++ programs, you can specify build types such as "Release" or "Debug". Release mode applies optimizations for faster execution speed, while Debug mode adds debugging information to facilitate debugging with a debugger.

To specify these build types when building with Catkin, use the --cmake-args option.

For example: ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

for release mode build, or: ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug

for debug mode.

These should be specified in addition to the options set in :ref:`ros_build_choreonoid_cmake_options`.

Choreonoid-related ROS packages are set to Release by default, but some packages don't set the build type to Release by default, and your own packages might not have this setting. In such cases, optimizations won't be applied and the execution speed of built programs will significantly decrease. If you might build such packages, it's good to specify Release build using the above method.

.. _ros_catkin_build_command:

Building
--------

So far we've explained :ref:`ros_build_choreonoid_cmake_options` and :ref:`ros_catkin_config_cmake_build_type`.
If you're unsure about the detailed options, use the following settings for now.

**For Ubuntu 20.04 (ROS Noetic Ninjemys)** ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DCMAKE_BUILD_TYPE=Release

**For environments before Ubuntu 18.04 (ROS Melodic Morenia)** ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DUSE_PYTHON3=OFF -DCMAKE_BUILD_TYPE=Release

Once configuration is complete, let's build. You can build from any directory within the workspace with the following command: ::

 catkin build

For details on building, please refer to the `Catkin Command Line Tools manual <https://catkin-tools.readthedocs.io/en/latest/index.html>`_.

On successful build, you'll see a message like:

.. code-block:: none

 [build] Summary: All 4 packages succeeded!

.. note:: In Emacs, you can build using the "M-x compile" command, and this functionality can also be used in the Catkin environment. However, Catkin's output is normally colorized, but in Emacs the control codes are displayed, making the display hard to read. To avoid this, when executing "M-x compile", enter "catkin build --no-color" as the build command. The "--no-color" option disables the control codes for Catkin output coloring, eliminating display issues. You can also add the "-v" option to use "catkin build -v --no-color" to see the actual commands (compilation options, etc.) during build.

Note that :ref:`ros_catkin_build_command` can also be set using the --cmake-args option with catkin build. For example: ::

 catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

makes this build use Release mode. This way, you can switch only the build type for each build.

Furthermore, using the Profile feature of Catkin Command Line Tools, you can register settings as profiles in advance and specify a profile during build to switch entire option combinations. This method is explained in :doc:`catkin-profile`.

.. _loading_catkin_workspace_setup_script:

Loading the Workspace Setup Script
----------------------------------

When you build, a file called "setup.bash" is generated in the workspace's devel directory. The settings described in this script are necessary when running packages in the workspace, so configure it to be executed by default. Typically, add the following to the .bashrc file in your home directory: ::

 source $HOME/catkin_ws/devel/setup.bash

This will automatically execute this file when a terminal is launched, loading the settings.

Since these settings haven't been loaded yet during the first build, either restart the terminal or directly enter the above command from the command line to apply the settings.

.. note:: This script is **different from** the ROS setup.bash introduced in :doc:`install-ros`, so please be careful. Both scripts need to be loaded for packages in the workspace to function properly.

Supplement: Using Multiple Choreonoid Environments Together
-----------------------------------------------------------

Here we've introduced how to install Choreonoid that operates in the ROS environment (Catkin workspace). As mentioned at the beginning, Choreonoid can also be installed independently of ROS. However, when using them together on the same OS, caution is required.

When the ROS environment setup script is loaded into the system, the corresponding ROS (Catkin) directories are added to the shared library path. (They are added to the LD_LIBRARY_PATH environment variable.) In this state, if there are multiple shared libraries with the same name on the system, the ones in the ROS environment will typically be loaded with priority. If this is applied to software originally installed independently of ROS, libraries with different versions or build settings may be loaded, causing the software to malfunction. Mixing multiple environments is very dangerous.

To avoid this, it's safer to disable the loading of the "setup.bash" scripts mentioned in :ref:`loading_catkin_workspace_setup_script` and :doc:`install-ros` when using software independent of ROS. You can disable this by commenting out the relevant parts of .bashrc and then restarting the OS or terminal.

For Choreonoid, the RPATH information embedded in executable and shared library files allows execution without mixing with libraries built in other environments. This feature is enabled by default for executables and libraries generated in the build directory. (However, for relatively recent Ubuntu versions, `this update <https://github.com/choreonoid/choreonoid/commit/7f7900c3ec945f9da97b0e2ee484c1ddfe63d978>`_ or later is required.) Also, by setting CMake's ENABLE_INSTALL_RPATH to ON, this is enabled for files installed via "make install".

Since the above update, CMake has added an option called ENABLE_NEW_DTAGS. This is OFF by default, but turning it ON gives priority to LD_LIBRARY_PATH information over RPATH, increasing the risk of mixing. Please keep this option OFF unless specifically needed.

While Choreonoid has mechanisms to prevent shared libraries from mixing as much as possible, mixing can still occur depending on environment settings, and libraries might mix in other software used with Choreonoid. Therefore, as a general principle beyond just Choreonoid, when the same software is installed in multiple environments on the same OS, it's very important to use them in a way that prevents mixing to avoid issues.