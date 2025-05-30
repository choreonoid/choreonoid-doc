=======================
Plugin Development Basics
=======================

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

Overview
--------

This section explains the basics of creating plugins.

The procedure for creating a plugin can be formally expressed as follows:

1. Select the build form
2. Create the plugin source directory
3. Create source files and implement your own plugin class that inherits from :doc:`plugin-class` using the C++ language
4. Write build procedures using CMake
5. Build the plugin
6. Install if necessary
7. When you start Choreonoid, the created plugin will be loaded and you can use its functionality

In the following sections, we will explain the basic aspects of plugin creation along these steps to help you understand the overview of Choreonoid plugin development.

.. _plugin-dev-basics-build-forms:

Build Forms
-----------

Choreonoid is written in C++, and plugins must also be written as C++ programs. Therefore, when creating a plugin, you must first create C++ source files and compile them to generate executable binaries. There are basically two forms for proceeding with this build process:

1. Build within the Choreonoid main body build environment
2. Build independently from the Choreonoid main body

In case 1, the plugin is built as one of the components included in the Choreonoid main body build environment. If you are building the Choreonoid main body from source code yourself, you can build additional plugins in this form. As explained in :doc:`introduction`, many of the features provided by the Choreonoid main body are also implemented as plugins. Form 1 is to build your own plugins in the same way as those built-in plugins. Many existing plugins are designed to be built in form 1, and in most cases, choosing this option will be problem-free.

On the other hand, in form 2, the plugin is built independently from the Choreonoid main body. While this is a common form for building software modules, Choreonoid has not yet fully developed this form. In particular, when developing on Windows using Visual Studio, this form is not yet available. Therefore, this form is currently only available for Linux. This form is necessary when combining with the Choreonoid main body installed as a binary package or when combining with frameworks that require specific build procedures. An example of a plugin that actually adopts this form is :doc:`../ros/rosplugin`.

Currently, form 1 is more applicable, so this development guide will mainly proceed with explanations using form 1. We would like to continue developing binary packages for the Choreonoid main body and build scripts to make form 2 easier to choose in the future.

.. _plugin-dev-basics-source-directory:

Creating the Source Directory
-----------------------------

Once you have decided on the plugin build form, next create a directory to store the plugin source files. This is usually created one per plugin.

The location to create the source directory is as follows:

1. When building within the Choreonoid main body build environment

  Create a directory for the plugin in a subdirectory called "ext" included in the Choreonoid main body source directory.

2. When building independently from the Choreonoid main body

  Create a directory for the plugin at any location where you have access rights.

As shown above, in case 1, the location to create is fixed, so please be careful. When developing multiple plugins, you can create multiple source directories under ext. Note that by specifying "ADDITIONAL_EXT_DIRECTORIES" in the CMake settings when building the main body, you can add directories to place plugin source directories.

In case 2, you can create the source directory at any location. However, if a Choreonoid main body build environment exists, make sure to create it outside of that environment.


Creating Source Files
---------------------

As mentioned above, plugin source files are written in C++. By using the classes and functions of the C++ library provided by the Choreonoid main body, you can write C++ programs that function as plugins.

The development environment consisting of C++ libraries provided by the Choreonoid main body is called the Choreonoid SDK. The Choreonoid SDK consists of multiple libraries (modules), and the following are the foundational ones:

* Util library: Provides various utility classes/functions that do not depend on Choreonoid's GUI
* Base module: Provides classes and functions that form the foundation of Choreonoid's GUI

Here, Util is referred to as a "library" because it is a general-purpose component that can be used by other programs. On the other hand, Base corresponds to the core of Choreonoid and is only used within Choreonoid, so it is referred to as a "module". Both take the form of shared libraries (dynamic link libraries).

Plugins are also implemented based on these libraries/modules. Among them, :doc:`plugin-class` included in the Base module is the starting point for implementing Choreonoid plugins. In terms of normal C language programs, it is like the main function.

When creating a new plugin, first define a class that inherits from the Plugin class. In its constructor, describe the basic information of the plugin, and describe the initialization process in the initialize function. The minimum required class definition for this is expressed as follows: ::

 class FooPlugin : public Plugin
 {
 public:
    FooPlugin();
    virtual bool initialize() override;
 };


Here we define a plugin class with the name FooPlugin. In addition to the default constructor, we also define an initialize function that performs initialization. In the initialize function, we perform the process of registering newly added functions with the Choreonoid main body.

The functions to be registered can be freely implemented using C++. By using the libraries of the Choreonoid SDK, you can coordinate with the Choreonoid framework and existing functions. For example, for robot-related processing, you can use the following libraries:

* Body library: Provides robot-related functions that do not depend on the GUI
* BodyPlugin library: Provides robot-related functions that involve the GUI

Of course, it is also possible to use standard libraries and external libraries.

Also, Choreonoid requires C++11 or higher as the C++ version, and you can use features from C++11 onwards in plugin development. If your OS and development environment support it, you can also use newer C++ versions such as C++14 or C++17.

Store the source files created in this way in the plugin source directory.

Writing Build Procedures with CMake
-----------------------------------

Choreonoid uses CMake as its build system, and CMake is typically used to describe build procedures in plugin development as well. CMake has been adopted in the development of many software projects in recent years and has become a common build system, so there should be no particular problems with its use.

In CMake, build procedures are described in a file called CMakeLists.txt, and this is the same for Choreonoid plugin development. Basically, create one CMakeLists.txt per plugin and store it in the plugin source directory.

In CMakeLists.txt for plugins, in addition to normal CMake variables and commands, you can also use variables and commands provided by Choreonoid. By appropriately combining these, you can describe the plugin build procedure.

The way to write CMakeLists.txt differs slightly depending on the build form mentioned above. When building within the Choreonoid main body build environment, write it as part of the Choreonoid main body CMake project and directly use the functions and variables defined in the main body's CMakeLists.txt. On the other hand, when building independently from the Choreonoid main body, after importing the CMake package file provided by the Choreonoid main body, write it as an independent CMake project.

The description method for CMakeLists.txt is explained separately in :doc:`sdk-cmake`.

Building
--------

Once you have created the necessary source files and CMakeLists.txt, proceed with the build.

When building within the Choreonoid main body build environment, the plugin will be built at the same time when you build the Choreonoid main body in the normal way. For information on how to build the Choreonoid main body, please refer to the following pages:

* :doc:`../install/build-ubuntu`
* :doc:`../install/build-windows`

When building independently from the Choreonoid main body, create a CMake build directory for the plugin source directory, run CMake to generate build files. This is the same procedure as for general programs built with CMake.

When the build is successful, the plugin binary file is generated. This is called a shared library on Linux and a dynamic link library on Windows, with file extensions .so and .dll, respectively. Usually, a prefix is added to the file name to identify it as a Choreonoid plugin. If the plugin name is FooPlugin:

* libCnoidFooPlugin.so (Linux)
* CnoidFooPlugin.dll (Windows)

The file names will be something like these.

Installation
------------

The built plugin is stored in the Choreonoid main body plugin directory. The plugin directory is located at:

* [Choreonoid main body build/install destination]/lib/choreonoid-x.y

Here, x.y is where the Choreonoid main body version number goes.

When building within the Choreonoid main body build environment, the plugin binary file is stored in the plugin directory at the same time as the main body build/installation.

When building independently from the Choreonoid main body, the plugin installation must also be performed on the plugin side. Usually, you write this in CMakeLists.txt and install it using the build system's installation function.

Loading and Usage
-----------------

.. highlight:: text

When the plugin file is stored in the plugin directory, the plugin is loaded when Choreonoid starts. In that case, the following message is output to the message view: ::

 Plugin file "C:\choreonoid\choreonoid-1.8\CnoidFooPlugin.dll" detected.

 ...

 
 Foo plugin has been loaded.

By confirming this message, you can see that the plugin has actually been loaded.

If there is a problem loading the plugin, an error message is usually displayed, so if the created plugin is not functioning, first check the message view to see if loading has failed.

If the plugin loads successfully, the functions implemented in the plugin become available.