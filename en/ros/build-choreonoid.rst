Building Choreonoid related packages
====================================

In this section, we will build and install Choreonoid as a package in the ROS environment. It also builds some Choreonoid related packages.

In this document, Choreonoid is installed in a different way from :doc:`.../install/build-ubuntu` . Please note that even if you have already installed Choreonoid in that procedure, you will have to install Choreonoid for ROS separately from it. It is also possible to use the Choreonoid installed by the normal procedure in the ROS environment, but the necessary ROS packages and documents are currently being prepared for that. For the time being, please follow the instructions in this document to install Choreonoid for ROS. The Choreonoid to be installed here should be managed independently from the Choreonoid installed in the normal procedure.

.. contents::
   :local:

.. highlight:: sh

.. _ros_make_catkin_workspace:

Create a Catkin workspace
--------------------------

Create a Catkin workspace for Choreonoid.

The workspace is usually created in the home directory. The workspace is usually named "catkin_ws", but you may change this name, in which case replace "catkin_ws" with that name in the following instructions.

First, create an empty workspace. ::

 mkdir catkin_ws
 cd catkin_ws
 mkdir src
 catkin init

.. note:: Here we use the command "catkin init" to initialize the workspace, but there is also a command "catkin_init_workspace" that does the same thing. The former command is part of Catkin's new command system, `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ , while the latter is an older form of Catkin's commands. Since Choreonoid's ROS integration assumes the use of Catkin Command Line Tools, the old-style Catkin command (catkin_make) is used. The ROS integration of Choreonoid assumes the use of Catkin Command Line Tools, so please do not use the old-style Catkin commands (catkin_make, etc.).

.. _ros_choreonoid_add_package_sources:

Add package sources
-------------------

Clone the source code repositories of Choreonoid itself and the ROS plugin into the "src" directory of the created workspace. ::

 cd src
 git clone https://github.com/choreonoid/choreonoid.git
 git clone https://github.com/choreonoid/choreonoid_ros.git

Each of them corresponds to the following Github repositories.

* `choreonoid <https://github.com/choreonoid/choreonoid>`_ : Choreonoid itself
* `choreonoid_ros <https://github.com/choreonoid/choreonoid_ros>`_ : ROS package to use ROS features in Choreonoid

.. note:: The ROS integration function of Choreonoid is currently available for the Choreonoid development version. The latest development version can be used by cloning the above repository.

Also, if you want to refer to the explanations in the next and subsequent sections, you should clone the samples used in the explanations. ::

 git clone https://github.com/choreonoid/choreonoid_ros_samples.git
 git clone https://github.com/choreonoid/choreonoid_joy.git

Each of them corresponds to the following Github repositories.

* `choreonoid_ros_samples <https://github.com/choreonoid/choreonoid_ros_samples>`_ : samples for using ROS with Choreonoid
* `choreonoid_joy <https://github.com/choreonoid/choreonoid_joy>`_ : A ROS node for using a joystick (gamepad) with the Choreonoid mapping.

Please keep the contents of each repository as up-to-date as possible.

.. note:: As for ROS-related packages for Choreonoid, there are also `choreonoid_ros_pkg <https://github.com/fkanehiro/choreonoid_ros_pkg>`_ and `choreonoid_ rosplugin <https://github.com/s-nakaoka/choreonoid_rosplugin>`_ . Since these are not officially supported at present, please do not install them when using Choreonoid in the way described in this manual. If they are installed as binary packages or included in the workspace of catkin, the functions described in this document may not work properly. If you want to follow the descriptions in this manual, please create a new workspace and install only the specified packages first to check the behavior.


Using the repository management tool
------------------------------------

There are two tools for managing multiple repositories at once: `wstool <http://wiki.ros.org/wstool>`_ and `vcstool <https://github.com/dirk-thomas/vcstool>`_ . By using these tools, you can update multiple repositories at once, so it would be a good idea to use them.

Here is a brief introduction to vcstool. vcstool can be installed by ::

 sudo apt install python-3-vcstool

For usage, see ::

 vcs help

for how to use it.

In the directory above each repository, type ::

 vcs pull

will perform a git pull on all repositories, bringing them all up to date.

Installing dependent packages
-----------------------------

Install the dependency packages that are required to build and run Choreonoid.

Go to the source directory of Choreonoid and execute the corresponding script ::

 misc/script/install-requisites-ubuntu-20.04.sh

for Ubuntu 20.04.

For Ubuntu 18.04 and 16.04, execute

* misc/script/install-requisites-ubuntu-18.04.sh
* misc/script/install-requisites-ubuntu-16.04.sh

respectively.

This process should be solved by the dependency package information for Catkin, but there is a part of Choreonoid that is not yet complete, so you need to do this to ensure the installation.

If the latest version of Choreonoid has already been installed on the OS independently from ROS, this work should have already been applied and there is no need to perform it again.

.. _ros_build_choreonoid_cmake_options:

Setting CMake options
---------------------

If you want to set CMake options for building Choreonoid, use the "--cmake-args" option of the catkin config command.

First, run ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF


to prohibit the generation of the normal Choreonoid executable. In ROS, programs are invoked in the form of "nodes" instead of regular executables, and the node executable for Choreonoid is included in the choreonoid_ros package. Having both the regular executable and the node version of Choreonoid can be confusing, but the above options can avoid this.

It is also possible to enable optional plugins for Choreonoid. For example, if you want to use the "media plugin" for playing video and audio files on Choreonoid, you can do the following. ::

 catkin config --cmake-args -DBUILD_MEDIA_PLUGIN=ON

If you want to set multiple options, just enumerate the options. For example, the following command will allow you to set both disallow the generation of the regular executable and build the media plugin. ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DBUILD_MEDIA_PLUGIN=ON

.

After configuring, execute ::

 catkin config

and you will see the configuration of your workspace. If you see something like

.. code-block:: none

 Additional CMake Args: -DBUILD_CHOREONOID_EXECUTABLE=OFF -DBUILD_MEDIA_PLUGIN=ON

, it's OK.

.. note:: Setting it this way will enable these options for all packages in the workspace, and may cause other packages to have unintended options enabled. However, Catkin does not seem to have the ability to set CMake options for each package individually (`requested but not yet available <https://github.com/catkin/catkin_tools/issues/205>`_ ), so this is the only way to do it.

.. note:: The BUILD_MEDIA_PLUGIN option above is just an example for explanation, and is not necessarily required when using Choreonoid and ROS. If you do not need to play media files such as movies on Choreonoid, you do not need to turn on this option.

If you want to remove the options you have set, execute ::

 catkin config --no-cmake-args

If there are options you want to use in ROS environment, please enable them in the above way.

.. _note_on_ros_python_version:

Setting the Python version
^^^^^^^^^^^^^^^^^^^^^^^^^^

Choreonoid builds a Python plugin and a wrapper library for Python by default, but you need to be careful about the Python version to use. Choreonoid uses Python3 by default, but previous versions of ROS, specifically up to Melodic Morenica for Ubuntu 18.04, use Python2 (version 2.7). When using such ROS versions, Python3 in Choreonoid conflicts with Python2 in ROS, causing problems.

.. note:: Noetic Ninjemys, the ROS version for Ubuntu 20.04, now uses Python3, so the default settings should not cause any problems. For Ubuntu 20.04, please skip the following explanation.

For the previous ROS versions that use Python2, set Choreonoid to use Python2 as well. This can be done by turning off the USE_PYTHON3 option in CMake when building Choreonoid. If you do this, Python version 2 will be used in Choreonoid.

In catkin, this can be achieved by setting ::

 catkin config --cmake-args -DUSE_PYTHON3=OFF

.

Alternatively, if you do not need the Python feature of Choreonoid, you can turn off the Python feature itself as follows. ::

 catkin config --cmake-args -DENABLE_PYTHON=OFF -DBUILD_PYTHON_PLUGIN=OFF -DBUILD_PYTHON_SIM_SCRIPT_PLUGIN=OFF

.. _ros_catkin_build_type:

Set the build type
------------------

In general, when building a C/C++ program, you can specify the type of build, such as "Release" or "Debug"; in the case of Release (release mode), optimizations are applied to speed up execution, and in the case of Debug (debug mode), debugging information is provided to facilitate debugging with a debugger.

If you want to specify these build types when building on Catkin, you still need to use the --cmake-args option.

For example ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

to build in the release mode, or ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug

will put you in the debug mode.

These should be specified in addition to the options specified by :ref:`ros_build_choreonoid_cmake_options` .

Choreonoid-related ROS packages are set to Release by default, but some packages do not set the build type to Release by default, and some of your own packages may not have such a setting. In such a case, optimization will not be applied and the execution speed of the built program will be greatly reduced, so if you are likely to build such a package, it is better to specify Release build as described above.

.. _ros_catkin_build_command:

Build
------

If you are not sure about the detailed options, you can use the following settings for now.

**For Ubuntu 20.04 (ROS Noetic Ninjemys)** ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DCMAKE_BUILD_TYPE=Release

**For Ubuntu 18.04 (ROS Melodic Morenia) and earlier** ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DUSE_PYTHON3=OFF -DCMAKE_BUILD_TYPE=Release

After the configuration is complete, let's build. If the directory is in your workspace, you can build it with the following command. ::

 catkin build

For details on how to build, see the `Catkin Command Line Tools manual <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ .

If the build is successful, you will see the

.. . code-block:: none

 [build] Summary: All 4 packages succeeded!

.. note:: In Emacs, you can use the "M-x compile" command to build, and you can also use this feature in the Catkin environment. However, the output of Catkin is usually colored, but the control code is displayed in Emacs, which makes it difficult to see the output as it is. To avoid this, you can enter "catkin build --no-color" as a build command when executing "M-x compile." By including "--no-color", the control code for coloring Cakin output will be disabled and the display will not be distorted. You can also add the "-v" option and use "catkin build -v --no-color" to see the actual commands (compile options, etc.) at build time.

.. _ros_catkin_cmake_build_type:


Note that :ref:`ros_catkin_build_command` can also be set by the --cmake-args option to catkin build. For example ::

 catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

to build in the Release mode for this build. In this way, you can switch only the build type for each build.

.. In addition, using the profile feature of Catkin Command Line Tools, you can register each setting as a profile in advance, and specify the profile when building to switch the entire combination of options. This method is described in :doc:`catkin-profile` .

.. _loading_catkin_workspace_setup_script:

Loading the workspace setup script
----------------------------------

When you build, a file named "setup.bash" will be generated in the devel directory of your workspace. The settings described in this script are required to run the packages in the workspace, so it should be run by default. Normally, in the .bashrc file in your home directory, add ::

 source $HOME/catkin_ws/devel/setup.bash

. This file will be automatically executed when the terminal is invoked, and the configuration will be loaded.

The first time you build the packages in the workspace, the settings will not be loaded yet, so you will need to reboot the terminal or type the above command directly from the command line to reflect the settings.

.. note:: Please note that this script is **different from** the setup.bash of ROS itself, which was installed with :doc:`install-ros` . Both scripts need to be loaded in order for the packages on the workspace to work properly.

Tip: About using multiple Choreonoid environments together
----------------------------------------------------------

In this section, we have introduced how to install Choreonoid running on the ROS environment (Catkin workspace). As mentioned at the beginning, Choreonoid can be installed independently from ROS. However, when they are used together on the same OS, care must be taken.

When the setup script of the ROS environment is loaded into the system, the corresponding directories of ROS (Catkin) are added to the shared library path list. (This will be added to the LD_LIBRARY_PATH environment variable.) In this state, if there are multiple shared libraries with the same name on the system, the one in the ROS environment will usually be loaded first. If this is applied to software that was originally installed independently of ROS, libraries with different versions and build settings will be loaded, and the software may not work properly. This means that it is very dangerous to mix multiple environments.

To avoid this, it is safe to disable the inclusion of the setup.bash script described in :ref:`loading_catkin_workspace_setup_script` and :doc:`install-ros` above when using software that is independent of ROS. It can be disabled by commenting out the relevant part of the .bashrc and then rebooting the OS or terminal.

Choreonoid can be executed without mixing with libraries built in other environments by using the RPATH information embedded in the executable and shared library files. This feature is enabled by default for executables and libraries that are generated in the build directory. (However, for relatively newer Ubuntu versions, it is necessary to use `this update <https://github.com/choreonoid/choreonoid/commit/7f7900c3ec945f9da97b0e2ee484c1ddfe63d978>`_ or later.) It is also enabled for files installed by "make install" by setting CMake's ENABLE_INSTALL_RPATH to ON.

Since the above update, CMake has added an option called ENABLE_NEW_DTAGS. This option is OFF by default, but if you turn it ON, the information in LD_LIBRARY_PATH will take precedence over RPATH, increasing the risk of confusion. If you do not need this option, please leave it OFF.

As described above, Choreonoid has a mechanism to prevent the shared libraries from being mixed up as much as possible. Nevertheless, depending on the environment settings, there is a possibility that some libraries may get mixed up, and there is also a possibility that libraries may get mixed up in other software that is used in conjunction with Choreonoid. Therefore, when the same software is installed in multiple environments on the same OS, it is very important to use it in such a way that it does not get mixed up, not only with Choreonoid but also with other software.
