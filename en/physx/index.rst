
PhysX Plugin
============

.. contents::
   :local:
   :depth: 1

Overview
--------

About PhysX
~~~~~~~~~~~

PhysX is a physics engine developed by NVIDIA. While it was originally introduced as a high-speed physics computation engine that runs on NVIDIA's GPUs, the parts that run on the CPU are now open source and are published on `GitHub <https://github.com/NVIDIA-Omniverse/PhysX>`_ under the BSD 3-Clause license. It is a general-purpose physics engine that handles not only rigid bodies, articulated mechanisms, and contact computation but also simulation of cloth, soft bodies, and particles, and it is used in a wide range of fields including games, robotics, and CG.

One of PhysX's main features is its computational speed. PhysX has an implementation that has been refined over many years for applications where real-time performance is important, and it runs fast thanks to SIMD instructions, parallelization, and efficient contact computation. It also has excellent scalability when the number of objects in a scene increases, and it is designed to continue computing at a practical speed even in situations where many rigid bodies are in contact with each other.

For the simulation of articulated multi-body robots, a method based on reduced-coordinate formulation called **Articulation Reduced Coordinate** is used. This is the same approach as the AIST simulator, Choreonoid's standard physics engine, which solves the equations of motion in the robot's joint space, and it can handle articulated robots as stably as the AIST simulator.

About the PhysX Plugin
~~~~~~~~~~~~~~~~~~~~~~

Choreonoid's PhysX plugin is a plugin for using PhysX as a physics engine in Choreonoid. It provides a "PhysX Simulator" as a simulator item, which can be used in the same manner as simulator items for other physics engines.

This plugin primarily targets the simulation of robot models with rigid bodies and joints. Standard models used in Choreonoid can be used as is, and position control, velocity control, and torque control are all supported.

The plugin also provides **PxContinuousTrack** as a device for realizing continuous track mechanisms. Rather than treating the many shoes that make up a track as individual rigid bodies, this device treats them as a continuous belt wrapped around driving wheels and idler wheels, reproducing the behavior in which the track shoes actually circulate around the wheels according to wheel rotation and grip the ground. Compared to the approach of treating each shoe as a rigid body, it significantly reduces computational cost while enabling stable simulation of the characteristic ground contact behavior of tracks. This is a feature unique to the PhysX plugin and is particularly useful for the simulation of tracked robots.

PhysX is positioned as a quasi-standard physics engine in Choreonoid, and its source code is bundled with the Choreonoid main body's repository. This means that PhysX does not need to be downloaded and built separately, and it becomes available automatically just by building Choreonoid.

Setup
-----

Basic Build Method
~~~~~~~~~~~~~~~~~~

The source code of the PhysX plugin is bundled with the Choreonoid main body and is enabled by default. Therefore, simply following the normal Choreonoid build procedure will build the PhysX plugin together and make it ready for immediate use.

The following two CMake options are provided, but normally there is no need to change them.

* ``ENABLE_PHYSX`` : Builds the bundled PhysX library. The default is ``ON``.

* ``BUILD_PHYSX_PLUGIN`` : Builds the PhysX plugin itself. The default follows the value of ``ENABLE_PHYSX`` (so it is normally ``ON``).

If you do not want to use PhysX, set both ``ENABLE_PHYSX`` and ``BUILD_PHYSX_PLUGIN`` to ``OFF`` in the CMake configuration.

.. note::

  Although the default value of ``BUILD_PHYSX_PLUGIN`` follows the value of ``ENABLE_PHYSX``, this only works "when no value is saved in the cache" due to how CMake works. Therefore, if you once configure CMake with ``ENABLE_PHYSX=ON`` and then change only ``ENABLE_PHYSX`` to ``OFF``, ``BUILD_PHYSX_PLUGIN`` will remain at the cached value ``ON``, which causes an error as it is then treated as the path for using an external PhysX. Either explicitly set both to ``OFF``, or re-create the build directory before configuring.

Supplement: Using an Externally Built PhysX
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to use a PhysX library that you have built yourself rather than the bundled PhysX, the procedure is to first build an external PhysX SDK and then perform the CMake configuration on the Choreonoid side as follows. This can be used when there is a problem with the bundled PhysX, or when you want to try a newer version of PhysX.

The source code of the PhysX SDK is published at `NVIDIA-Omniverse/PhysX (GitHub) <https://github.com/NVIDIA-Omniverse/PhysX>`_.

Note that the procedure described here is for a Linux environment (operation confirmed on Ubuntu 24.04).

**Installing Dependency Packages**

The following packages are required to build the PhysX SDK.

.. code-block:: sh

  sudo apt install cmake build-essential python3 curl

**Obtaining the PhysX SDK Source**

Obtain the PhysX SDK source code from GitHub.

.. code-block:: sh

  git clone https://github.com/NVIDIA-Omniverse/PhysX.git

If you want to use a specific version, switch to its release tag. For example, to use the same version as the bundled PhysX, do the following.

.. code-block:: sh

  cd PhysX
  git checkout 107.3-physx-5.6.1

**Disabling the Snippets Build**

The default configuration also builds the PhysX samples (Snippets), but they may fail to build. Since they are not necessary for building the library used by Choreonoid, disable them.

Open the following file in a text editor.

.. code-block:: text

  physx/buildtools/presets/public/linux-gcc-cpu-only.xml

Change the value of ``PX_BUILDSNIPPETS`` from ``"True"`` to ``"False"``.

.. code-block:: xml

  Before: <cmakeSwitch name="PX_BUILDSNIPPETS" value="True" .../>
  After:  <cmakeSwitch name="PX_BUILDSNIPPETS" value="False" .../>

**Building the PhysX SDK**

Move to the physx directory and generate the build project.

.. code-block:: sh

  cd physx

Specify the preset using one of the following methods.

Method A: Specifying directly via command-line arguments:

.. code-block:: sh

  ./generate_projects.sh linux-gcc-cpu-only

Method B: Selecting from the interactive menu:

.. code-block:: sh

  ./generate_projects.sh

In this case, a list of presets will be displayed, so select ``linux-gcc-cpu-only``.

Next, move to the generated release build directory and build.

.. code-block:: sh

  cd compiler/linux-gcc-cpu-only-release
  make -j$(nproc)

When the build completes, static library files are generated under ``PhysX/physx/bin/linux.x86_64/release/``.

**CMake Configuration for Choreonoid**

On the Choreonoid side, configure CMake to use the external PhysX. When invoking CMake in the build directory, configure it using one of the following methods.

Method A: Specifying directly from the command line:

.. code-block:: sh

  cmake <path to Choreonoid source directory> \
        -DENABLE_PHYSX=OFF \
        -DBUILD_PHYSX_PLUGIN=ON \
        -DPHYSX_DIR=<path to the physx directory of PhysX> \
        -DPHYSX_BINARY_TYPE=release

For ``PHYSX_DIR``, specify the absolute path of the ``physx`` directory in the PhysX repository cloned in the procedure above (for example, ``-DPHYSX_DIR=$HOME/PhysX/physx``).

Method B: Configuring interactively with ``ccmake``:

.. code-block:: sh

  ccmake <path to Choreonoid source directory>

On the ccmake screen, configure the following items.

* Change ``ENABLE_PHYSX`` to ``OFF``
* Change ``BUILD_PHYSX_PLUGIN`` to ``ON``
* Enter the absolute path of PhysX's ``physx`` directory in ``PHYSX_DIR``
* Enter ``release`` for ``PHYSX_BINARY_TYPE``

After configuration, apply the settings with the ``[c]`` key and generate the Makefile with the ``[g]`` key.

Once the CMake configuration is complete, build Choreonoid as usual.

``PHYSX_BINARY_TYPE`` is a variable that specifies the build type of the PhysX SDK, and the following four types are available.

* ``release`` : Optimized build for normal use. The fastest.
* ``profile`` : Optimized + profiling functionality + PVD support
* ``checked`` : Optimized + argument checking of API calls
* ``debug`` : No optimization + debug information inside PhysX

Unless there is a specific reason, specify ``release``. The type you specify must be built in advance on the PhysX SDK side.

Samples
-------

Sample projects that use the PhysX plugin are provided in the ``choreonoid/sample/PhysX/`` directory.

PxTank
~~~~~~

:file:`sample/PhysX/PxTank.cnoid` is a sample of track simulation using a tank-shaped robot model. It uses the continuous track simulation feature unique to the PhysX plugin, and you can observe a robot with a track wrapped between multiple idler and driving wheels running on both flat ground and uneven terrain.

For the track, unlike the common approach of treating each shoe as a rigid body, the PhysX plugin provides a dedicated mechanism that treats the track as a single virtual belt. This mechanism expresses the ground contact state of the track as contact forces while keeping the computational cost down and achieving stable simulation.

When you open this project and press the simulation run button, the tank will automatically start moving.

PxCrawlerJoystick
~~~~~~~~~~~~~~~~~

:file:`sample/PhysX/PxCrawlerJoystick.cnoid` is a sample for operating a tracked mobile robot with a joystick. It uses the track simulation feature in the same way as PxTank, but differs in that the user can operate it in real time using a gamepad or other joystick.

During simulation, you can move the robot forward and backward and turn it by operating the sticks of the connected joystick.
