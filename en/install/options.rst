Optional Features
=============

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

This section introduces the optional features that can be selected in Choreonoid's build configuration.

Optional features are enabled/disabled through CMake settings. For information on this operation, please refer to :ref:`build-ubuntu-cmake` in :doc:`build-ubuntu` or :ref:`build-windows-cmake` in :doc:`build-windows`.

.. highlight:: sh

Main Options
------------

* **ENABLE_GUI**

 Enables features implemented on Choreonoid's GUI. This is ON by default and is an essential option when using Choreonoid as application software. If you want to use only the library parts that do not depend on the GUI, you can reduce build time by setting this to OFF.

* **BUILD_ASSIMP_PLUGIN**

 Builds the Assimp plugin. Assimp is a library for loading various 3D model files, and this plugin enables loading of COLLADA, Blender, X, and DXF format files. For building this on Windows, please refer to :ref:`build-windows-options`. This is ON by default on Ubuntu.

* **ENABLE_FREE_TYPE**

  Enables text rendering on the scene view using the FreeType library. In the current version of Choreonoid, when this option is ON, distance values are displayed numerically on the scene view when using the distance measurement function. This is ON by default on Ubuntu. On Windows, this is OFF by default, and the FreeType library must be installed separately for use.

* **BUILD_POSE_SEQ_PLUGIN**

 Builds the PoseSeq plugin. This plugin provides choreography functionality using key poses.

* **BUILD_BALANCER_PLUGIN**

 Builds the Balancer plugin. This plugin enables automatic balance correction in the choreography functionality. When creating choreography for bipedal walking robots, this feature can be used to create motions that (theoretically) do not fall over.

* **BUILD_MOCAP_PLUGIN**

 A plugin for loading and displaying motion capture data. Currently supports BVH format motion capture data.

* **ENABLE_PYTHON**

 Enables Choreonoid's Python bindings. Python bindings make it possible to use Choreonoid libraries implemented in C++ from Python. For example, by importing the "cnoid.Body" module, which is the Python binding of the Body library, you can write various robotics-related processes centered on the Body class in Python.

 Additionally, when ENABLE_GUI is ON, the Python plugin and PythonSimScript plugin for using Python on Choreonoid's GUI are also built together.

 This is ON by default on Ubuntu. For enabling this on Windows, please refer to :ref:`build-windows-options`.

* **BUILD_ODE_PLUGIN**

 Builds the ODE plugin. This plugin enables the use of the open-source dynamics calculation library "Open Dynamics Engine (ODE)" as a physics engine for simulation. To use this, `Open Dynamics Engine (ODE) <http://www.ode.org/>`_ must be installed. The Ubuntu package installation script also installs ODE, so you can build by simply turning this option ON. For Windows builds, please refer to :ref:`build-windows-options`. Note that even without the ODE plugin, simulation using the built-in AIST physics engine is possible.

* **BUILD_AGX_DYAMICS_PLUGIN**

 Builds the AGX Dynamics plugin. AGX Dynamics is a commercial physics engine developed by Algoryx in Sweden, enabling simulation using this engine. For details, please refer to :doc:`../agxdynamics/index`.

* **BUILD_MULTICOPTER_PLUGIN**

 Builds the Multicopter plugin. This plugin enables multicopter simulation. For details, please refer to :doc:`../multicopter/index`.

* **BUILD_SCENE_EFFECTS_PLUGIN**

 Builds the Scene Effects plugin. This plugin enables rendering of effects such as fire and smoke on scenes.

* **BUILD_MEDIA_PLUGIN**

 Builds the Media plugin. This plugin enables playback of various media files including video and audio on Choreonoid.

* **BUILD_TRAFFIC_CONTROL_PLUGIN**

 Builds the TrafficControl plugin. This plugin enables simulation of various communication failures such as communication delays, bandwidth limitations, and packet loss. For details, please refer to :doc:`../trafficcontrol/index`.

* **BUILD_FCL_PLUGIN**

 Builds the FCL plugin. This plugin enables the use of the open-source collision detection library `Flexible Collision Library (FCL) <https://github.com/flexible-collision-library/fcl>`_ for collision detection. The Ubuntu package installation script also installs FCL, so you can build by simply turning this option ON. On Windows, you need to install FCL yourself. Note that even without the FCL plugin, the built-in collision detection functionality is available, so there are no particular issues.


Sample-Related Options
-----------------------

The following are options for building sample data and programs.

* **ENABLE_SAMPLES**

 Enables samples. This enables building of basic samples, and also allows building of additional samples with the following options. This is ON by default.

* **BUILD_SIMPLE_CONTROLLER_SAMPLES**

 Builds various simulation samples implemented with Simple Controller. This is ON by default.

* **BUILD_SUBMERSIBLE_SAMPLE**

 A sample for simple simulation of underwater robots.

* **BUILD_WRS2018**

 A sample version of the competition model from the "Tunnel Accident Response and Recovery Challenge" at the international robot competition World Robot Summit 2018 held in 2018. For details, please refer to :doc:`../wrs2018/index`.

Extension Framework Options
----------------------------

The following are options that serve as the foundation for extension features. There is no need to enable them unless specifically required.

* **BUILD_MANIPULATOR_PLUGIN**

 A plugin that serves as the foundation for implementing manipulator teaching and simulation functionality on Choreonoid.

* **ENABLE_CORBA**

 Enables CORBA-related functionality. This is required for using OpenRTM and OpenHRP-related features. It is implemented using `omniORB <http://omniorb.sourceforge.net/>`_. On Ubuntu, you can install the necessary omniORB-related packages with the following command: ::

  sudo apt install libomniorb4-dev libcos4-dev omniidl omniorb-nameserver python-omniorb omniidl-python

 Note that this option is not normally displayed in menu-style CMake configuration tools. It becomes visible when switching to Advanced Mode.

Experimental/Development Stage Feature Options
-----------------------------------------------

The options introduced below are for features still in experimental/development stages and may not necessarily work properly. These are mainly options used for development of the corresponding features. In menu-style CMake configuration tools, these options are normally not displayed and only become visible when switching to Advanced Mode. (Please note that some content in the build method pages for each plugin introduced below may be outdated.)

* **BUILD_BULLET_PLUGIN**

 Builds the Bullet plugin. This plugin enables the use of the open-source physics calculation library `Bullet Physics <https://github.com/bulletphysics/bullet3>`_ in Choreonoid's simulation functionality. For build methods, please refer to :doc:`build-bullet-plugin`.
 
* **BUILD_PHYSX_PLUGIN**

 Builds the PhysX plugin. This plugin enables the use of the physics calculation library `PhysX <https://developer.nvidia.com/gameworks-physx-overview>`_ in Choreonoid's simulation functionality. For build methods, please refer to :doc:`build-physx-plugin`.

* **BUILD_ROKI_PLUGIN**

 Builds the ROKI plugin. This plugin enables the use of the robot kinematics library `RoKi <https://github.com/zhidao/roki>`_ in Choreonoid's simulation functionality. For build methods, please refer to :doc:`build-roki-plugin`.

* **BUILD_SPRINGHEAD_PLUGIN**

 Builds the Springhead plugin. This plugin enables the use of the dynamics calculation library "Springhead" as a calculation engine for Choreonoid's simulation functionality. For details about Springhead, see the `Springhead homepage <http://springhead.info/wiki/>`_, and for build methods, please refer to :doc:`build-springhead-plugin`.

* **BUILD_SDF_PLUGIN**

 Builds the SDF plugin. This plugin enables loading of models described in Simulation Description Format (SDFormat). The implementation uses the `SDFormat library <https://github.com/osrf/sdformat>`_. For building on Ubuntu, you can install the necessary libraries with the following command: ::

  sudo apt install libsdformat6-dev libogre-1.9-dev

* **ENABLE_LUA**

 Enables bindings and script execution functionality using the `Lua programming language <http://www.lua.org/>`_. For building on Ubuntu, install Lua-related packages with the following command: ::

  sudo apt install lua5.3 iblua5.3-dev lua-posix
 
Other Options
-------------

Choreonoid has other options besides those listed above, but if you don't understand what they are, you should basically not turn them ON.