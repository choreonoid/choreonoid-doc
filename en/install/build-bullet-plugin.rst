Building the Bullet Plugin
=====================

.. contents:: Table of Contents
   :local:


Installing Bullet Physics Library
----------------------------------

To use BulletPlugin, you need to install the Bullet Physics Library.

First, obtain the source code of Bullet Physics Library.

Display the `Bullet Physics Library <http://bulletphysics.org>`_ site and click on the Download text in the upper left to navigate to the site where Bullet's source code is stored.
Here, click on **Source code (zip)** or **Source code (tar.gz)** to download files in each compressed format, then extract them to an appropriate location.

The currently confirmed working version is bullet 2.87.

There should be a file called README.md in the extracted directory. The installation method is described there, so basically follow those instructions. There are some differences depending on the Bullet version, but recent versions can use cmake, so you can install it using the same procedure as building choreonoid. Here we introduce the installation method for version 2.87.

For Ubuntu
~~~~~~~~~~

Use cmake. It should already be installed since it was used when installing choreonoid.

Move to the extracted directory and enter and execute the following: ::

 ./build_cmake_pybullet_double.sh

This will execute everything from cmake to build.

A directory called build_cmake will be created, so move there. ::

 make install
 
Execute this to install. If you want to specify the installation destination, in build_cmake run ::
 
 ccmake .

This will start cmake, so specify the installation destination in the **CMAKE_INSTALL_PREFIX** item before running make install. Other options can also be changed here. **USE_DOUBLE_PRECISION** must be set to ON.

For Windows
~~~~~~~~~~~

The README states to execute build_visual_studio_vr_pybullet_double.bat, so you can use this, but here we want to use the familiar cmake.

First, move to the directory where bullet was extracted and create a directory called build_cmake.

Start the cmake GUI and enter the path to the directory where bullet was extracted in the "where is the source code" field, and the path to the created build_cmake in the "where is build the binaries" field.

Turn ON the following options:

* **BUILD_EXTRAS**
* **INSTALL_EXTRA_LIBS**
* **INSTALL_LIBS**
* **USE_DOUBLE_PRECISION**
* **USE_MSVC_RUNTIME_LIBRARY_DLL**

Also, it's safer to turn OFF the following options:

* All **BUILD_XXX_DEMOS**
* **BUILD_BULLET3**
* **BUILD_PYBULLET**
* **BUILD_PYBULLET＿XXX**
* **BUILD_UNIT_TESTS**

Set **CMAKE_BUILD_TYPE** to Release, and set the installation destination with **CMAKE_INSTALL_PREFIX**.

Similar to choreonoid's :ref:`build-windows-cmake`, press the **"Configure"** and **"Generate"** buttons to proceed.

.. note:: There seem to be differences in CMake options depending on the Bullet version. Please consider the explanation here as an example for the target version.

A Visual Studio solution file should be created in build_cmake, so open it.

Similar to choreonoid's :ref:`build-windows-visualstudio`, confirm that the display shows **"Release"** and **"x64"**, then execute **"Build Solution"** and **"INSTALL"**.

Building the Plugin
-------------------

When building choreonoid, in the CMake settings, set the **BUILD_BULLET_PLUGIN** item to "ON" and specify the Bullet library installation destination in **BULLET_DIR**.

Running Simulations
-------------------

Simulations using the Bullet plugin follow the same method as :ref:`using other physics simulators<simulation_creation_and_configuration_of_simulator_item>`. It becomes executable by creating a simulator item "Bullet Simulator" and placing it as a child item of the world item.