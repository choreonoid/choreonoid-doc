================================
Minimum Plugin Sample (S01)
================================

.. contents:: Table of Contents
   :local:

Overview
--------

In this section, we will actually create a plugin to show how the concepts explained in :doc:`basics` are implemented in specific procedures and code.
The plugin to be created will be the minimum necessary to achieve this purpose.

The plugin name is "DevGuide Plugin". The development guide will continue to extend this sample plugin while maintaining this name.

Selecting the Build Form
------------------------

For the :ref:`build form <plugin-dev-basics-build-forms>`, we will first proceed with the "Build within the Choreonoid main body build environment" form. Therefore, please first build the Choreonoid main body from source code. We will use that build environment to build the plugin as well.

The "Build independently from the Choreonoid main body" form will be introduced at the end of this section.

Creating the Source Directory
-----------------------------

.. highlight:: sh

Since we are building within the Choreonoid main body build environment, we create the plugin source directory in the main body's ext directory. Let's name the directory "DevGuidePlugin" to match the plugin name. To do this from the command line: ::

 cd Choreonoid main body source directory/ext
 mkdir DevGuidePlugin

Of course, you can also do the same thing using GUI tools like a file explorer.

Creating Source Files
---------------------

.. highlight:: cpp

Create the following source file in the above directory: ::

 #include <cnoid/Plugin>
 #include <cnoid/MessageView>
 
 using namespace cnoid;
 
 class DevGuidePlugin : public Plugin
 {
 public:
     DevGuidePlugin() : Plugin("DevGuide")
     {
 
     }
 
     virtual bool initialize() override
     {
         MessageView::instance()->putln("Hello World!");
         return true;
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)

Since it's a C++ program, the extension is usually .cpp. Please store it in the source directory with the filename "DevGuidePlugin.cpp".

Explanation of the Source File
-------------------------------

Let's explain the contents of this source file.

Including Headers
~~~~~~~~~~~~~~~~~

This sample includes the following two headers: ::

 #include <cnoid/Plugin>
 #include <cnoid/MessageView>

These headers are included in the Choreonoid SDK. Headers of the Choreonoid SDK are basically stored in a subdirectory called "cnoid", and are specified with cnoid as a prefix like this. Also, like the C++ standard library, headers are specified without extensions.

The following is an overview of the headers included here:

* **cnoid/Plugin**

  The header where the Plugin class is defined. Source files that define custom plugins must include this header. This allows you to define a custom plugin class that inherits from the Plugin class.

* **cnoid/MessageView**

  The header corresponding to the Choreonoid main body's message view, where the MessageView class is defined. Include this header when using the message view in your plugin.


Namespace
~~~~~~~~~

All classes and functions included in the Choreonoid SDK are defined within the "cnoid" namespace. The following declaration imports this namespace: ::

 using namespace cnoid;

This declaration allows us to omit the namespace specification cnoid for the following classes used in this sample:

* cnoid::Plugin
* cnoid::MessageView

Of course, namespaces are meant to avoid name conflicts, and it's not good to use the using directive carelessly. In principle, it's better to avoid using the using directive in header files and write out the full namespace specification. On the other hand, in implementation files (.cpp), if name conflicts are not a problem, you can simplify the code by using the above declaration.

In this guide, sample source files will basically be implemented with this declaration to keep the description concise.


Plugin Class Definition
~~~~~~~~~~~~~~~~~~~~~~~

The custom DevGuide plugin is defined with the following code: ::

 class DevGuidePlugin : public Plugin
 { 
     ...
 };


Choreonoid plugins are defined as classes that inherit from the Plugin class. The class name must end with "Plugin". Following this principle, the rest of the class name can be freely determined. However, it's also necessary to ensure that the name doesn't conflict with other plugins, such as those included with the Choreonoid main body or additional plugins installed in the same environment.

Constructor
~~~~~~~~~~~

You must first define a constructor in the plugin class. This is done in the following part: ::

 DevGuidePlugin() : Plugin("DevGuide")
 {
 
 }

The base Plugin class doesn't have a default constructor and only has a constructor that takes the plugin name as an argument. Therefore, you must use the initializer list to provide the plugin name to the Plugin class. This ensures that the plugin name is always set.

Note that the name set here should be the plugin class name without the "Plugin" part. Therefore, we set the string "DevGuide" instead of "DevGuidePlugin" here.

As for other processes to write in the constructor, there is the description of plugin dependencies. If a plugin depends on other plugins, you need to inform Choreonoid about that plugin. This can be done with the require function. For example, if this plugin depends on the Body plugin: ::

 DevGuidePlugin() : Plugin("DevGuide")
 {
     require("Body");
 }

Since we don't have dependencies on other plugins yet, we haven't written anything in the constructor. Plugins that depend on other plugins will be introduced later in this guide.

initialize Function
~~~~~~~~~~~~~~~~~~~

Plugin initialization is usually written in the initialize function: ::

 virtual bool initialize() override
 {
     ...
 }

The initialize function is defined as a virtual function in the base Plugin class, and by overriding it, each plugin's initialization process is executed.

Each plugin's initialize function is executed in an order that considers the dependencies between plugins after the plugin is loaded and the constructor is executed. When initialization is successful and the plugin's functionality is ready to use, this function should return true. This lets the Choreonoid main body recognize that the plugin is ready to use. If initialization fails, it should return false.

This plugin outputs text to the message view with the following code written in this function: ::

 MessageView::instance()->putln("Hello World!");

Here, we get the message view instance (pointer) with the instance function of the `MessageView class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1MessageView.html>`_, and output text with a newline using the putln function. This displays "Hello World!" in the message view when the plugin is initialized. This process itself has no particular meaning, but we're doing it as a first step to execute the process written in the plugin.

As in this example, plugins can obtain objects held by the Choreonoid main body and use them to perform various processes. Choreonoid has various functions besides the message view, and by utilizing them, you can realize the functionality you want to provide with your plugin. If you're already using Choreonoid, you should be aware of what functions Choreonoid has. In many cases, there are corresponding headers and libraries in the Choreonoid SDK, and by including or linking them, they can be used from plugins.

Note that the Plugin class defines the following virtual functions in addition to the initialize function. These functions are also intended to be overridden and can be used in plugin implementation.

* **virtual bool finalize()**

  Write the plugin's termination process. When Choreonoid terminates, if you need to destroy objects used by the plugin or release system resources, write them in this function.

* **virtual const char* description()** 

  A function that returns the plugin's description. By overriding this function, you can set a description for your custom plugin. The set description can be checked from the Choreonoid main menu "Help" - "About Plugins". When publishing a developed plugin externally, write the plugin overview, copyright, and license conditions here.

Plugin Entry Definition
~~~~~~~~~~~~~~~~~~~~~~~

Finally, we have the following description: ::

 CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)

Here we use a macro called "CNOID_IMPLEMENT_PLUGIN_ENTRY" defined in the Plugin header. When you write the plugin class name in this macro, it defines a function to get the plugin instance from the plugin DLL. If you don't write this description, the created DLL won't be recognized as a plugin, so don't forget it.

Note that each plugin must be created as one DLL that implements one plugin. You cannot implement multiple plugins in one DLL (you cannot write the above macro more than once), so please be careful.

.. _plugin-dev-cmake-description-basics:

Writing Build Procedures with CMake
-----------------------------------

.. highlight:: cmake


The next thing to do is create a file for building with CMake. CMake basically describes build procedures in a file called CMakeLists.txt. Create a file with the following content with the filename CMakeLists.txt in the plugin source directory.

Write the following line in CMakeLists.txt: ::

 choreonoid_add_plugin(CnoidDevGuidePlugin DevGuidePlugin.cpp)

choreonoid_add_plugin is a command defined in the Choreonoid main body's CMakeLists.txt that sets up the plugin target. Using this command: ::

 choreonoid_add_plugin(target name source file)

By writing it like this, you can build the plugin binary from the specified source file. You can specify multiple source files here.

We're setting the plugin name. The plugin name should start with "Cnoid" and end with "Plugin" like this. Here we set this name in a variable called target so it can be used in subsequent commands. While it's not necessary to set it in a variable, doing so centralizes the plugin name setting. ::


This command is a customized version of CMake's built-in add_library command. Plugins are created as shared libraries (dynamic link libraries) and are a type of library. Therefore, you can also build plugins using the add_library command for creating libraries.

However, there are various rules for building and using it as a Choreonoid plugin. Therefore, you need to set several parameters for add_library, and you need to use several other commands for configuration. Writing all of this using only CMake's built-in commands would be time-consuming and error-prone. Therefore, Choreonoid defines the choreonoid_add_plugin command to enable plugin building with minimal description.

The choreonoid_add_plugin command basically performs the equivalent of: ::

 add_library(target name source file)

internally, so you can perform additional build settings for the defined target. For example, if the plugin has dependent libraries, write as follows: ::

 choreonoid_add_plugin(target name source file)
 target_link_libraries(target name PUBLIC library name)

If you think of replacing the choreonoid_add_plugin part with add_library, this is the same as normal CMake usage. You can use other CMake commands for this target as needed.

.. note:: The "PUBLIC" part in the above "target_link_libraries" specifies that the symbols of the libraries specified after it should be exposed in the plugin binary (shared library). If you don't want to expose the library symbols, use "PRIVATE" here. This description is required from Choreonoid version 2.0, and you must specify either "PUBLIC" or "PRIVATE" - it cannot be omitted. (In Choreonoid versions before 2.0, conversely, you cannot specify it.) Note that you can specify multiple libraries after "PUBLIC" or "PRIVATE". Basically, you should specify "PUBLIC", but if the target library causes conflicts with other libraries, you can avoid problems by specifying "PRIVATE".


Thus, in this sample, we could describe the plugin build in just one line. By building within the Choreonoid main body build environment, such a concise description is possible. This is because we can share various processes and information described in the Choreonoid main body's CMakeLists.txt, which is an advantage of using this build form.

Note that it's good to add the following description at the beginning of the plugin's CMakeLists.txt: ::

 option(BUILD_DEVGUIDE_PLUGIN "Building a sample plugin of the plugin development guide" OFF)
 if(NOT BUILD_DEVGUIDE_PLUGIN)
   return()
 endif()

This description adds an option called "BUILD_DEVGUIDE_PLUGIN" to the CMake configuration. Here the default is set to OFF, in which case the build of this plugin is skipped. If you want to build the plugin, turn this option ON in the CMake configuration. Having the ability to switch whether to build the plugin like this may make plugin development and operation easier. Especially when publishing the plugin source code externally, including such a description will improve usability for users.

.. note:: For details on how to write CMakeLists.txt, refer to the `CMake manual <http://www.cmake.org/cmake/help/help.html>`_. Also, by reading the CMakeLists.txt of libraries, other plugins, and samples included in Choreonoid, you'll understand the general writing style. The choreonoid_add_plugin command is written in cmake/ChoreonoidBasicBuildFunctions.cmake in the Choreonoid source, so if you want to know the implementation details of this command, please refer to that.

Building and Installation
-------------------------

.. highlight:: text

Following the steps so far, the plugin directories and files should be as follows: ::

 + Choreonoid main body source directory
   + ext
     + DevGuidePlugin (plugin source directory)
       - CMakeLists.txt
       - DevGuidePlugin.cpp

In this state, build the Choreonoid main body. Then the CMakeLists.txt existing in DevGuidePlugin under ext will be automatically detected and incorporated into the Choreonoid main body's CMake processing. As a result, this plugin will be built along with the Choreonoid main body. Of course, the build process only targets areas that need updating, so if the Choreonoid main body build is already complete, that part of the build process will be skipped. Therefore, this method doesn't particularly increase build time.

.. note:: Avoid having CMake process the CMakeLists.txt in the DevGuidePlugin directory directly. The above CMakeLists.txt isn't written to be processed that way, and doing so will create extra temporary build files in the Choreonoid main body source directory, potentially preventing the main body from building normally.

After a successful build, perform the installation. This can also be done by executing the main body installation operation. On Linux, you can execute binaries from the build environment directly, so just building is OK. On Windows, an installation operation is required, so don't forget to execute it.

If you've successfully built and installed the plugin, the plugin binary file should be stored in Choreonoid's plugin directory. On Windows, check if there's a file called CnoidDevGuidePlugin.dll under lib/choreonoid-x.y in the installation destination. On Linux, there should be a file called libCnoidDevGuidePlugin.so under lib/choreonoid-x.y in the build directory or installation destination.

Loading the Plugin and Checking Execution Results
--------------------------------------------------

Let's start Choreonoid.
Plugin files existing in Choreonoid's plugin directory are automatically loaded when Choreonoid starts, so the DevGuide plugin we created should also be loaded.

You can check whether the plugin is actually loaded by the text displayed in the message view. First, when a plugin file is detected: ::

 Plugin file "C:\choreonoid\usr\lib\choreonoid-2.0\CnoidDevGuidePlugin.dll" detected.

A message like this is displayed. Furthermore, when this file is successfully loaded and plugin initialization is complete: ::

 DevGuide plugin has been loaded.

This message is displayed.

Since the plugin we created displays text in the message view in the initialization function, it should actually display: ::

 Hello World!
 DevGuide plugin has been loaded.

This confirms that the contents of the initialization function are actually being executed.

While this only displays a message and has no particular meaning, by implementing processes that add functionality in this part, you can extend functionality through plugins. This guide will explain methods to do this step by step.


Building Independently from the Choreonoid Main Body
----------------------------------------------------

Plugins can also be built independently from the Choreonoid main body. In that case, CMake is typically used for building as well. Below we introduce this method. Note that as mentioned in :doc:`basics` :ref:`plugin-dev-basics-build-forms`, this method is currently not applicable when using Visual Studio on Windows, so please be aware. The following is the procedure for use on Ubuntu Linux.

.. _plugin-dev-minimum-sample-sdk-setup:

Setting up the Choreonoid SDK
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When using this build method, the Choreonoid SDK must be in a state where it can be used externally. Therefore, if you're building the Choreonoid main body from source code, you need to do the following work in advance. (If you've installed Choreonoid as a binary package with SDK included, all of the following may already be satisfied.)

1. Turn ON the CMake INSTALL_SDK option when building the Choreonoid main body
2. Install with make install after build completion
3. Set the path to the Choreonoid SDK CMake files

For 1, it's ON by default on Linux, so it's fine if you haven't changed the settings. After building, also perform the installation work shown in 2. For details, see :ref:`build-ubuntu_install` in :doc:`../install/build-ubuntu`.

3 is necessary to obtain CMake information needed for building Choreonoid-related programs from outside. If the installation destination is the default /usr/local, the path to the CMake files is also set by default, so this work is not necessary. However, if you're installing in another directory, configuration is required.

.. highlight:: sh

Configuration is done using the environment variable CMAKE_PREFIX_PATH. For example, if you've installed the Choreonoid main body in the choreonoid directory in your home directory: ::

 export CMAKE_PREFIX_PATH=~/choreonoid

If CMAKE_PREFIX_PATH is already set for other software: ::

 export CMAKE_PREFIX_PATH=~/choreonoid:$CMAKE_PREFIX_PATH

Add the path like this.

The Choreonoid SDK CMake files are actually placed under share/choreonoid/cmake in the installation destination. This placement method is one of CMake's standard placement methods, and the CMake files in this directory become search targets with the above settings.

.. note:: Be careful if you have multiple Choreonoid main bodies installed on your OS. If paths to CMake files and executables are set for each of those multiple Choreonoid instances, an unexpected instance may be targeted during plugin building or Choreonoid execution, leading to unexpected behavior during building or execution. This can happen even if the user is not aware of it, when self-building from source, installation by package system, use in special environments like ROS, etc. are mixed. Differences in versions, build options, enabled plugins, etc. between those multiple instances can also be problematic. Therefore, when using Choreonoid, be careful to target only one instance at a time. Note that there's no problem using multiple Choreonoid instances simultaneously if they are properly separated.

Creating the Source Directory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a source directory for the plugin. When building within the Choreonoid main body build environment, we created the source directory in the Choreonoid main body's ext directory, but this time we need to create it in a different location, so be careful. Even if there's a Choreonoid main body source directory, create it in an independent location. As long as this is observed, you can create it anywhere with any name.

For example, suppose the Choreonoid main body source directory is in src/choreonoid in the home directory, and create a directory src/DevGuidePlugin. Create the source file and CMakeLists.txt there as well. Then the directory structure will be as follows: ::

 + src
   + choreonoid (Choreonoid main body source directory)
   + DevGuidePlugin (plugin source directory)
     - CMakeLists.txt
     - DevGuidePlugin.cpp

※ This is just an example, and you don't have to follow the same structure.

Writing CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~

.. highlight:: cmake

When building a plugin independently from the Choreonoid main body, additional description is required in the CMakeLists.txt for plugin building. For this sample, write as follows: ::

  cmake_minimum_required(VERSION 3.10)
  project(DevGuidePlugin)
  find_package(Choreonoid REQUIRED)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  
  choreonoid_add_plugin(CnoidDevGuidePlugin DevGuidePlugin.cpp)

This time, the plugin's CMakeLists.txt will be processed by CMake independently. The above description includes all the necessary descriptions for building based on this. Below we explain these descriptions: ::

 cmake_minimum_required(VERSION 3.10)

Specifies the minimum required CMake version. By specification, CMakeLists.txt to be processed by CMake must first have this description. Since features are added or removed with each CMake version, you need to specify an appropriate version based on your CMake version and the CMakeLists.txt description. For reference, the CMake versions installed with standard packages on Ubuntu Linux are as follows:

* Ubuntu 16.04: CMake version 3.5.1
* Ubuntu 18.04: CMake version 3.10.2
* Ubuntu 20.04: CMake version 3.16.3

Ubuntu 16.04's support ended in April 2021. It might be good to use CMake version 3.10 from Ubuntu 18.04, which is still supported, as a guideline.

Note that the actual CMake version used can be any version newer than what's specified here. ::

 project(DevGuidePlugin)

Sets the project name. This must also be included in CMakeLists.txt processed independently. There are no particular rules for naming projects, but for plugins, simply setting the plugin name would be clear and good. ::

 find_package(Choreonoid REQUIRED)

find_package is a standard CMake command for obtaining information about external software libraries. If the target software provides information in CMake format (if CMake files for that purpose are installed), you can obtain information with this command. Since we need this, we needed to set the path to the Choreonoid SDK CMake files.

This command is used as: ::

 find_package(target software name)

For Choreonoid, specify Choreonoid here. If detection is successful, a true value is set in the variable Choreonoid_FOUND. Specifying REQUIRED as an option makes the target package required. In this case, if detection fails, an error is output and subsequent CMake processing stops. Since we're trying to create a Choreonoid plugin, Choreonoid is required, so we specify REQUIRED.

When the Choreonoid SDK is detected, the following variables are set as related information:

.. list-table::
 :widths: 40,60
 :header-rows: 1

 * - Variable
   - Content
 * - CHOREONOID_INCLUDE_DIRS
   - SDK header file directories
 * - CHOREONOID_LIBRARY_DIRS
   - SDK library file directories
 * - CHOREONOID_UTIL_LIBRARIES
   - Libraries to link when using the Util library
 * - CHOREONOID_BASE_LIBRARIES
   - Libraries to link when using the Base module
 * - CHOREONOID_PLUGIN_DIR
   - Directory to install plugin files

Several other variables and functions for building Choreonoid-related programs are also defined. Actually, since most of the descriptions for building can be done with those functions, there aren't many cases where you need to use the variables defined here. ::

  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})

Specifies the C++ version to use. Here CMAKE_CXX_STANDARD is a built-in CMake variable, and when you put the C++ version number here, it adds options to the compiler to use that version. And CHOREONOID_CXX_STANDARD is the C++ version used when building the Choreonoid main body. Considering binary compatibility between the Choreonoid main body and plugins, it's desirable to match the C++ versions as much as possible, so we include this description. ::

 choreonoid_add_plugin(CnoidDevGuidePlugin DevGuidePlugin.cpp)

This part is the same as when building within the Choreonoid main body build environment. This description performs the basic processing to build the plugin. The choreonoid_add_plugin command is defined along with the Choreonoid SDK detection above. In essence, we've been setting up the SDK and adding descriptions to CMakeLists.txt to be able to use this command.

Building and Installation
~~~~~~~~~~~~~~~~~~~~~~~~~

.. highlight:: sh

Build and install independently from the Choreonoid main body. In other words, just use CMake normally targeting this plugin. For example, assuming the above directory structure, if working from the command line: ::

 cd src/DevGuidePlugin
 mkdir build
 cd build
 cmake ..
 make
 make install

If find_package fails during cmake execution, the following error message is output:

.. code-block:: text

 CMake Error at CMakeLists.txt:3 (find_package):
   By not providing "FindChoreonoid.cmake" in CMAKE_MODULE_PATH this project
   has asked CMake to find a package configuration file provided by
   "Choreonoid", but CMake did not find one.
 
   Could not find a package configuration file provided by "Choreonoid" with
   any of the following names:
 
     ChoreonoidConfig.cmake
     choreonoid-config.cmake
 
   Add the installation prefix of "Choreonoid" to CMAKE_PREFIX_PATH or set
   "Choreonoid_DIR" to a directory containing one of the above files.  If
   "Choreonoid" provides a separate development package or SDK, be sure it has
   been installed.

If this output appears, there's a problem with :ref:`plugin-dev-minimum-sample-sdk-setup`, so please check that.

For make execution, as explained in :ref:`install_build-ubuntu_build`, it's good to perform parallel builds according to the number of CPU cores (threads). For example, with a logical 8-core CPU: ::

 make -j8

This will build in parallel using up to 8 processes, making the build faster.

Also, when building plugins in this form, installation is always required. The installation destination is recognized by information acquisition through find_package, but if it's a directory that cannot be written without root privileges: ::

 sudo make install

Install with root privileges.


Other Build Methods
-------------------

.. highlight:: sh

So far we've introduced methods for building with CMake. As such, CMake is typically used for Choreonoid plugin development. This is because CMake is currently widely used and has become a common build tool, and CMake can automate various settings and processes related to building.

However, it's not necessarily the case that you can only build with CMake. If you must build using other methods due to special circumstances, that's also possible. In that case, consider the following points:

* Ensure access to header files (include directories) of the Choreonoid SDK and other dependent libraries
* Link to necessary libraries for the Choreonoid SDK and other dependent libraries
* Build with settings (compile/link options) compatible with the Choreonoid main body and dependent library binaries

These are general considerations when developing software modules written in compiled languages like C++. When using CMake, you don't have to consider these things much, but when using other methods, you first need to accurately understand these matters.

For necessary information for this, please refer to this development guide and the Choreonoid source code. Also, for the Choreonoid main body and plugins already built with CMake, you can display all commands actually used in building by adding the VERBOSE setting to the make command during building: ::

 make VERBOSE=1

This output can be said to contain almost all the information necessary for building, so it may be good to refer to this.