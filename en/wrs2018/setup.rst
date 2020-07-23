Structure of the simulation environment
=======================================

.. contents::
   :local:

.. highlight:: sh

Preparing the simulation PC
---------------------------

First, prepare the simulation PC and install Choreonoid.

In the actual competition, Ubuntu 16.04 was used as OS, but if you want to run the samples, it will work with the latest environment. In the following, we will assume the use of Ubuntu 18.04.

Note that you should use a native install of Ubuntu. It’s not that it won’t work on a virtual machine, but the simulation may be slow or other problems may occur.

Installing Git
--------------

You will need a Git of the version control system in order to proceed with the work described below. If you don’t have it installed yet, use the following command to install it. ::

 sudo apt install git

.. _wrs2018_install_agx:

Installing AGX Dynamics
-----------------------

If you have an AGX Dynamics license, you should install AGX Dynamics in advance. Download the package for the corresponding Ubuntu version from the AGX Dynamics download site indicated by the vendor. Also, if you have been provided with a USB dongle, insert it into the PC.

Once the package has downloaded, install it by following the instructions for :doc:`../agxdynamics/install/install-agx-ubuntu` .

If you don’t have an AGX Dynamics license, skip this task.

.. _wrs2018_install_openrtm:

Installing Choreonoid
---------------------

Following the `Choreonoid latest version (development version) manual  <../index.html>`_  section on `Building and installing from source code (Ubuntu Linux version) <../install/build-ubuntu.html>`_, install the latest `development version  <../install/build-ubuntu.html#id4>`_ of Choreonoid.

First, get the Choreonoid source code from the Git repository. ::

 git clone https://github.com/choreonoid/choreonoid.git

Move to the directory where the source code is saved. ::

 cd choreonoid

Install the dependency packages. ::

 misc/script/install-requisites-ubuntu-18.04.sh

Configure build settings with CMake If you are using only the default features of Choreonoid, run the command ::

 cmake .

However, in order to execute the WRS2018 sample, the following options must also be enabled (ON).

* WRS2018 sample

 * BUILD_WRS2018

* If you are using AGX Dynamics

 * BUILD_AGX_DYNAMICS_PLUGIN
 * BUILD_AGX_BODYEXTENSION_PLUGIN

* When reproducing smoke and flames

 * BUILD_SCENE_EFFECTS_PLUGIN

* When using the multicopter

 * BUILD_MULTICOPTER_PLUGIN
 * BUILD_MULTICOPTER_SAMPLES

You can set these options interactively using the ccmake command, but you can also give the cmake command the -D option. For example, to set BUILD_SCENE_EFFECTS_PLUGIN to ON, input the following. ::

 cmake -DBUILD_SCENE_EFFECTS_PLUGIN=ON

This option can be added multiple times. If you want to enable all the above options, input the following. ::

 cmake -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON

If you do not have AGX Dynamics installed, remove the corresponding option from the above command line parameters and execute it.

Next, perform the build with the make command. ::

 make

If you are using a multi-core CPU, it is a good idea to parallelize the build by adding the -j option to the make command. For example, as follows. ::

 make -j 8

In this case, up to 8 build processes will be run simultaneously. It’s a good idea to input this if the CPU has 4 cores and 8 threads. Usually, specify the number of logical cores in the CPU.

Even after installation, you can always use the latest version of Choreonoid by executing the following commands in the source directory where the above operation was done. ::

 git pull
 make -j 8

Preparing the gamepad
---------------------

With this sample, you can operate the robot using a gamepad. To do so, prepare a gamepad and connect it to a PC.

For details about what gamepads can be used, refer to the  :doc:`../simulation/tank-tutorial/index` section on  :ref:`simulation-tank-tutorial-gamepad` . We recommend the  `DUALSHOCK4 <http://www.playstation.com/en-us/explore/accessories/gaming-controllers/dualshock-4/>`_ controller for PlayStation 4. The DUALSHOCK4 can be used wirelessly using a  `USB wireless adapter <https://support.playstation.com/s/article/DUALSHOCK-4-USB-Wireless-Adapter?language=en_US>`_ .
