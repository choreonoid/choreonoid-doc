Building Choreonoid-related Packages
====================================

This section explains how to build the ROS 2 packages necessary for integrating Choreonoid with ROS 2.
Note that ROS 2 packages for newly constructed robot systems using Choreonoid and ROS 2 can also be built using this method.


.. contents::
   :local:

.. highlight:: sh


Creating a ROS 2 Workspace
--------------------------

In ROS 2, we use a working directory called a "workspace" for introducing new programs and data.
Programs and data are constructed in the form of "packages" on the workspace.

Here we will create a new workspace for integration with Choreonoid.
The workspace is usually created in your home directory. Here, we'll name the workspace "ros2_ws". This name can be set freely. If you use a different workspace name, replace "ros2_ws" in the following explanations with your chosen name.

First, create an empty workspace: ::

   mkdir -p ~/ros2_ws/src
   cd ros2_ws

If you have an existing workspace, you may use that instead.

.. note:: Build work on ROS 2 workspaces typically uses a build system called "colcon". For a more detailed explanation of colcon, please refer to the following pages in the official ROS 2 documentation.

 * `ROS 2 Documentation: Jazzy - Using colcon to build packages <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_
 * `ROS 2 Documentation: Humble - Using colcon to build packages <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_

.. _add_choreonoid_package_sources_for_ros2:

Adding Package Sources
----------------------

To integrate Choreonoid with ROS 2, you need the choreonoid_ros ROS 2 package.

Clone this source repository into the "src" directory of the workspace: ::

   cd src
   git clone https://github.com/choreonoid/choreonoid_ros.git

In :doc:`install-choreonoid`, it was mentioned that Choreonoid main body can also be :ref:`installed as a ROS 2 package <ros2_install_choreonoid_install_as_ros2_package>`. In that case, clone the Choreonoid main body source repository into the src directory of this workspace as well: ::

  git clone https://github.com/choreonoid/choreonoid.git

※ If you have already installed Choreonoid main body using the :ref:`ros2_install_choreonoid_standard_method`, please do not perform this step.

If there are other additional ROS 2 packages you want to use in this environment, clone their source repositories here as well.

For example, if you clone the source repository for :doc:`ros2-mobile-robot-tutorial`, you'll be able to try this tutorial. In that case, do the following in the same src directory: ::

  git clone https://github.com/choreonoid/choreonoid_ros2_mobile_robot_tutorial.git



.. _install-choreonoid-ros2-dependencies:

Installing Dependencies for choreonoid_ros
------------------------------------------

The choreonoid_ros package depends on several ROS 2 packages, which also need to be installed. Install these dependency packages using the ROS 2 method. Specifically, execute the rosdep command as follows: ::

   rosdep install -y --from-paths ~/ros2_ws/src --ignore-src

This command will additionally install the dependency packages described in the "package.xml" of choreonoid_ros.
If there are other packages as well, their dependency packages will also be installed together.

.. note:: To resolve dependencies on Choreonoid main body, you need to :ref:`ros2_install_choreonoid_register_to_rosdep`. If you haven't done this, you can execute this command by adding the "--skip-keys choreonoid" option to the above rosdep command to temporarily ignore the dependency on Choreonoid main body.


.. _ros2_colcon_build_command:

Building
--------

.. Let's build. You can build with the following command from any directory within the workspace: ::

Let's build using the following command. The directory where you execute the command must be the top of the workspace: ::

   cd ~/ros2_ws
   colcon build --symlink-install

The build option `--symlink-install` installs various files using symbolic links during installation. This consumes less PC storage space since file copying doesn't occur, and for files that don't require compilation, edited content is reflected immediately. For example, in Choreonoid, .body files and .project files, and in ROS 2, .urdf files and .yaml files are subject to immediate reflection of edits.

For details on this command's options, please refer to `build - Build Packages <https://colcon.readthedocs.io/en/released/reference/verb/build.html>`_ in the `official colcon documentation <https://colcon.readthedocs.io/en/released/index.html>`_.

If the build is successful, messages like the following will be output:

.. code-block:: none

   Starting >>> choreonoid_ros
   Finished <<< choreonoid_ros [10.0s]

   Summary: 2 packages finished [10.0s]

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

Reference: Using Repository Management Tools
---------------------------------------------

By using the repository management tool `vcstool <https://github.com/dirk-thomas/vcstool>`_, you can perform operations like cloning and updating multiple repositories in bulk.

vcstool can be installed with the following command (it's installed with :ref:`ros2_install_ros2_install_dev_tools`): ::

   sudo apt install python3-vcstool

Check the usage with: ::

   vcs help

By executing: ::

 vcs pull

in a directory above each repository, git pull is executed for all repositories, allowing you to update all repositories to the latest version.

For example, with the following commands, you can update all clones in the "src" directory, including choreonoid and choreonoid_ros introduced in :ref:`add_choreonoid_package_sources_for_ros2`, to the latest version: ::

   cd ~/ros2_ws
   vcs pull src



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

Choreonoid-related packages are set to Release by default. However, in general, some packages may not set the build type to Release by default, and your own packages may not have this setting. In that case, optimization will not be applied, and the execution speed of the built program will be significantly reduced. If you might build such packages, it's good to specify Release build using the above method.

.. _ros2_build_choreonoid_as_ros2_package_note:

Reference: Notes When Building Choreonoid Main Body on the Workspace
---------------------------------------------------------------------

When you install Choreonoid main body using the :ref:`ros2_install_choreonoid_standard_method` and also :ref:`install as a ROS 2 package <ros2_install_choreonoid_install_as_ros2_package>`, you need to be careful about interference between them.

When the ROS 2 environment setup script is loaded into the system, the corresponding directory of the ROS 2 workspace is added to the shared library path. (It's added to the environment variable LD_LIBRARY_PATH.) In this state, if there are multiple shared libraries with the same name on the system, the ROS 2 environment ones will usually be loaded preferentially. If this is applied to software originally installed independently of ROS 2, libraries with different versions or build settings may be loaded, causing the software to malfunction. Mixing multiple environments is dangerous.

To avoid this, it's safer to disable the loading of the "setup.bash" scripts mentioned in :ref:`loading_ros2_workspace_setup_script` and :doc:`install-ros2` when using software independent of ROS 2. You can disable them by commenting out the relevant parts in the ".bashrc" configuration file and then restarting the OS or terminal.

Note that for Choreonoid, it's possible to execute without mixing with libraries built in other environments through RPATH information embedded in executable files and shared library files. This feature is enabled by default for executable files and libraries generated in the build directory. Also, by turning ON CMAKE's ENABLE_INSTALL_RPATH, this is also enabled for files installed by "make install".

Through such mechanisms, Choreonoid's shared libraries are designed to avoid mixing with those from other environments as much as possible. However, depending on the environment settings, they may still mix, and libraries may mix in other software used in conjunction with Choreonoid. Therefore, as a general matter not limited to Choreonoid, when the same software is installed in multiple environments on the same OS, it's very important to use them without mixing to avoid problems.

.. note:: If you turn ON the "ENABLE_NEW_DTAGS" option in CMake options when building Choreonoid, LD_LIBRARY_PATH information will be prioritized over RPATH, increasing the risk of mixing. If this option is not particularly necessary, please leave it at the default OFF.