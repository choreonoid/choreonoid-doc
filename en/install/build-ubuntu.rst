Building and Installing from Source Code (Ubuntu Linux)
=======================================================

While Choreonoid can be :doc:`installed via package <install-package-ubuntu>` on Ubuntu Linux, building from source code allows you to develop Choreonoid itself or achieve more flexible operation.
This document explains how to build Choreonoid from source code on Ubuntu Linux.

.. contents::
   :local:

.. highlight:: sh

Obtaining the Source Code
--------------------------

Development Version
~~~~~~~~~~~~~~~~~~~

Choreonoid development is conducted on `github <https://github.com/>`_, and the latest source code can be obtained from the following repository.

- https://github.com/choreonoid/choreonoid

The source code is managed as a `Git <http://git-scm.com/>`_ repository, and the git command is required for use. On Ubuntu, Git can be installed with the following command: ::

 sudo apt install git

The Choreonoid repository can be obtained by executing the following command: ::

 git clone https://github.com/choreonoid/choreonoid.git

This will create a directory called "choreonoid" containing the repository. Within this directory, you can update to the latest source code at any time by executing: ::

 git pull

For detailed information on using Git, please refer to Git manuals and tutorial articles.

Source code may be managed as a Git repository or otherwise. In either case, we will refer to the directory containing the source code as the **"source directory"**.

Release Version
~~~~~~~~~~~~~~~

The source code for Choreonoid release versions can be downloaded from the `Downloads <http://choreonoid.org/en/downloads.html>`_ page. Please download the appropriate version from the "Source Package" section on this page. The file is in ZIP format, so extract it in an appropriate directory with: ::

 unzip choreonoid-2.3.0.zip

After extraction, a directory such as choreonoid-2.3.0 will be created. This directory also becomes the **"source directory"** containing the source code.

.. note:: For release versions, the procedures in this manual targeting the development version may differ. For example, versions 2.0.0 and earlier also require installation of Boost C++ Libraries. For installation methods for release versions, please refer to the `manual for each release version <http://choreonoid.org/en/documents/index.html>`_.

Installing Development Tools and Dependent Software
----------------------------------------------------

Development Tools
~~~~~~~~~~~~~~~~~

To build Choreonoid from source code, the following development tools are required.

- Complete C/C++ standard development tools: A complete set of standard development tools including C/C++ compiler, Make, etc. is required. On Ubuntu, the complete set can be installed with the "build-essential" package. For C/C++ compilers, GCC is normally used, but Clang/LLVM is also available.
- `CMake <http://www.cmake.org/>`_: A build tool. It generates files for standard build tools such as Make or Visual Studio from its own descriptions. This enables efficient build descriptions that support many environments.

Dependent Libraries
~~~~~~~~~~~~~~~~~~~

The following libraries are also required for building basic functionality.

* `Eigen <https://eigen.tuxfamily.org>`_: A high-speed, high-functionality template library for matrix, vector, and linear algebra operations.
* `Qt <https://www.qt.io/>`_: A framework library including GUI toolkit.
* `gettext <http://www.gnu.org/s/gettext/>`_: Tools and libraries for multilingual display support.
* `fmtlib <https://github.com/fmtlib/fmt>`_: A library for outputting formatted strings.
* `libjpeg <http://libjpeg.sourceforge.net/>`_: A library for reading JPEG format image files.
* `libpng <http://www.libpng.org/pub/png/libpng.html>`_: A library for reading PNG format image files.
* `libzip <https://libzip.org/>`_: A library for reading and writing ZIP format files.
* `LibYAML <http://pyyaml.org/wiki/LibYAML>`_: A parser for YAML format text.
* `FreeType <http://freetype.org/>`_: A library for rendering fonts. Used for drawing text on 3D images.
* `Assimp <http://assimp.sourceforge.net/>`_: A library for reading various formats of 3D model files.

.. note:: Previous versions also depended on Boost C++ Libraries, but as of commit f40ea6fc on March 11, 2024, Boost C++ Libraries are no longer required. However, plugins distributed separately from Choreonoid itself may still require Boost C++ Libraries.

When building optional features, the following software may also be additionally required.

* `Python <https://www.python.org/>`_: Required when using the "Python plugin" for operating Choreonoid using the Python programming language. Python is usually installed by default, but development libraries are required when building the plugin.
* `Numpy <http://www.numpy.org/>`_: A Python library for various scientific and technical calculations. This is also required for the Python plugin.
* `Open Dynamics Engine (ODE) <http://www.ode.org/>`_: A physics calculation library. Required when using the "ODE plugin" for physics-based simulation.
* `GStreamer <http://gstreamer.freedesktop.org/>`_: A library for handling media files. Required when using the "Media plugin" for playing audio and video files in Choreonoid.
* `PulseAudio <http://www.freedesktop.org/wiki/Software/PulseAudio/>`_: A system for audio output. Installed by default on Ubuntu, but separate development libraries are required when building the Media plugin.
* `libsndfile <http://www.mega-nerd.com/libsndfile/>`_: A library for reading audio files. Required when using the Media plugin.

.. _build-ubuntu-install-packages:

Installing Dependent Packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

On Ubuntu, most of the above software can be easily installed using the script "install-requisites-ubuntu-x.x.sh" located under "misc/script". The x.x corresponds to the Ubuntu version. For example, for Ubuntu 24.04: ::

 misc/script/install-requisites-ubuntu-24.04.sh

When executed, you will be prompted for the sudo password. Enter it, and the necessary packages will be automatically installed via the package system.

Note that both Qt versions 6 and 5 are supported. Qt version 4 was supported up to Choreonoid 1.7, but is not supported in the latest version.

.. _build-ubuntu-cmake:

Build Configuration with CMake
-------------------------------

First, create a directory to use for building. In the Choreonoid source directory, create it with: ::

 mkdir build

This created directory is called the **build directory**. Here we use the directory name "build", but any name is acceptable. It is possible to create multiple build directories and build with different configurations for each.

Next, run CMake in the build directory: ::

 cd build
 cmake ..

This command performs compiler version checks, dependency library checks, etc., and generates the Makefiles necessary for building. Note that there are two periods after the cmake command, so be careful not to make mistakes here. This indicates that the source targeted by cmake is in the directory one level up.

If you have followed the above instructions on the target Ubuntu version, the Makefiles should be generated without problems. However, if required libraries are not installed in the specified locations, errors may occur when running cmake. In such cases, it is necessary to install them properly or modify the CMake build configuration. Build configuration can be done from the command line using the cmake command, or by running the ccmake command: ::

 ccmake ..

This allows various settings to be configured in a menu format. For details, please refer to the CMake manual.

Choreonoid also includes several optional features that are not built by default. An overview of these is summarized in :doc:`options`, so if you want any features, please enable them in the CMake configuration. For example, if you want to use simulation functionality with Open Dynamics Engine, set **BUILD_ODE_PLUGIN** to "ON".

Clang Configuration
-------------------

Normally, building is done using the GCC compiler, but it is also possible to build using Clang. In that case, install Clang and then set the environment variables CC and CXX, or set the CMake variables CMAKE_C_COMPILER and CMAKE_CXX_COMPILER.

Clang can be installed as follows: ::

 sudo apt install clang

To use C++, you also need the standard C++ library, but the above command alone may not install the clang-compatible standard C++ library. In that case, also install the standard C++ library package.

First, run: ::

 clang --verbose

And check the display such as: ::

 Selected GCC installation: /usr/bin/../lib/gcc/x86_64-linux-gnu/14

The last number shown here is the GCC version required for building with Clang. Install the standard C++ library corresponding to this version: ::

 sudo apt install libstdc++-14-dev

( `Reference page on stack overflow <https://stackoverflow.com/questions/74543715/usr-bin-ld-cannot-find-lstdc-no-such-file-or-directory-on-running-flutte>`_ )

When setting Clang usage with environment variables, set them as follows:

 * CC: clang
 * CXX: clang++

These need to be set when CMake is executed, so for example, when running CMake: ::

 CC=clang CXX=clang++ cmake ..

Or you can set them in advance: ::

 export CC=clang
 export CXX=clang++

Alternatively, since the above environment variables correspond to the CMake variables CMAKE_C_COMPILER and CMAKE_CXX_COMPILER: ::

 cmake -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=gcc ..

.. _install_build-ubuntu_build:

Building Choreonoid
--------------------

When CMake execution succeeds, a complete set of Makefiles for building is generated in the build directory. Execute: ::

 make

in the build directory to build Choreonoid.

For multi-core CPUs, build time can be shortened by performing parallel builds using the "-j" option. For example: ::

 make -j8

This will execute up to 8 build processes simultaneously. Normally, specifying the same number of processes as logical cores results in parallel building that makes maximum use of CPU capability.

Note that make with Makefiles generated by CMake does not display command details, and the build process is output in a clean, summarized display. This is very easy to read when checking build progress, but detailed compilation options given to GCC cannot be confirmed. When this is necessary: ::

 make VERBOSE=1

By turning on the VERBOSE variable like this, it is possible to output all command execution details.

You can also build using CMake commands instead of the make command. In this case: ::

 cmake --build build_directory

To perform parallel building: ::

 cmake --build build_directory --parallel parallel_count

If the parallel count is omitted, the compiler's default value is used. If you set the parallel count in the environment variable CMAKE_BUILD_PARALLEL_LEVEL, parallel building will be performed even without entering the --parallel option, so it is good to write this in .bashrc etc.

Also, adding the "-v" option will output command details similar to "make VERBOSE=1".

.. _build-ubuntu_install:

Installation
------------

When using Choreonoid on Ubuntu, it is possible to directly execute the executable file generated in the build directory. If the build is successful, an executable file called "choreonoid" will be generated under the "bin" directory in the build directory, so execute this: ::

 bin/choreonoid

If there are no problems with the build, Choreonoid's main window will start.

Since it is convenient to be able to execute without installation work, you may use this format if there are no particular problems.

On the other hand, it is also possible to install to a specified directory. In this case, only the binary files and data files necessary for software execution are collected in one place. Therefore, when sharing software system-wide, packaging it, or using it in conjunction with other software, installation work is performed.

To do this, execute: ::

 make install

in the build directory. This will install the complete set of files necessary for execution to the designated directory.

.. note:: Installation can also be executed with CMake commands: ::

  cmake --install build_directory

 This installs to the directory set in **CMAKE_INSTALL_PREFIX**. The installation destination can also be specified as: ::

  cmake --install build_directory --prefix installation_destination

On Ubuntu, the default installation destination is "/usr/local". Writing to this directory normally requires root privileges, so: ::

 sudo make install

is necessary.

For /usr/local, since the path is set by default to /usr/local/bin which stores executable files, you can execute Choreonoid simply by: ::

 choreonoid

regardless of the current directory location.

.. note:: Installing to the default installation destination /usr/local is **not recommended**. While this directory is common as a default installation destination, it should be considered as a convenience measure. When building and installing software from source code yourself, it is generally not managed by the OS package management system. This means you need to manage it yourself, but when such things are installed mixed together in the same directory /usr/local, it becomes very difficult to remove unnecessary files when upgrading specific software or to uninstall only specific software. Therefore, instead of installing to /usr/local, it is better to prepare dedicated directories for each software in the home directory and install there.


The installation destination can also be changed by configuring CMake's **CMAKE_INSTALL_PREFIX**. If multiple accounts do not need to use it, it might be good to set somewhere in the home directory as the installation destination. In this case, there is no need to sudo during installation.

However, in that case, the paths for executables and libraries will not be set. Also, CMake files for plugin development will not be in the search path. There is a shell script at the installation destination to solve this by setting up the necessary environment.

The script is a file called "setup.bash" at the installation destination. For example, if installed under "~/usr", it will be the file "~/usr/setup.bash". You can set it up by: ::

 source ~/usr/setup.bash

If you include the same line in your ".bashrc" file, this configuration will be set automatically.

Building Optional Features
---------------------------

Choreonoid includes several modules, plugins, samples, etc. other than those enabled by default in the above procedure. These are summarized in :doc:`options`.

The procedure for enabling optional features is basically as follows:

1. (If necessary) Install dependent libraries
2. Enable the corresponding option in CMake build configuration
3. Execute Choreonoid build again

For step 2, there are CMake variables corresponding to options, so set these to "ON".

Variables can be set from the command line with the cmake command, or from the menu screen displayed by the ccmake command.

For example, the "PoseSeq plugin" and "Balancer plugin" corresponding to Choreonoid's motion choreography functionality can be enabled as follows: ::

 cd build_directory
 cmake .. -DBUILD_POSE_SEQ_PLUGIN=ON -DBUILD_BALANCER_PLUGIN=ON

Conversely, to disable an option, set "OFF" to the corresponding variable. For example: ::

 cmake .. -DENABLE_SAMPLES=OFF

This configures samples not to be built.

Content set with the "-D" option is saved in the build directory, so it is possible to additionally set only the variables you want to change.
Of course, you can set multiple variables together, and you can perform all settings during cmake initialization.

By performing the build operation again after changing settings, optional features will be built and become available.

Other Environment Setup
------------------------

After completing the build and installation of Choreonoid itself, we recommend checking the following content to ensure a more comfortable usage environment:

* :doc:`setup-gpu`
* :doc:`setup-qt`
