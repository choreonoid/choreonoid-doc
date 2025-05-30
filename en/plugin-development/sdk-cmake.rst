===============
CMake Description Method
===============

.. list-table::
 :widths: 40,60
 :header-rows: 1

 * - Variable
   - Content
 * - CHOREONOID_DEFINITIONS
   - Compile options
 * - CHOREONOID_INCLUDE_DIRS
   - Header file directories
 * - CHOREONOID_LIBRARY_DIRS
   - Library file directories
 * - CHOREONOID_UTIL_LIBRARIES
   - Libraries to link when using Util module
 * - CHOREONOID_BASE_LIBRARIES
   - Libraries to link when using Base module
 * - CHOREONOID_PLUGIN_DIR
   - Directory to install plugin files

Next, the information obtained by find_package is used as follows: ::

 add_definitions(${CHOREONOID_DEFINITIONS})
 include_directories(${CHOREONOID_INCLUDE_DIRS})
 link_directories(${CHOREONOID_LIBRARY_DIRS})

With this description, compile options, include paths, and link paths are set appropriately. ::

 set(target CnoidHelloWorldPlugin)

Setting the plugin name in the variable target. ::

 add_library(${target} SHARED HelloWorldPlugin.cpp)

Since plugins become shared libraries, they can be built with CMake's standard add_library command.

In :ref:`hello-world-build-together`, we built the plugin with the choreonoid_add_plugin command which extends add_library, but when building a plugin standalone, we use add_library directly. ::

 target_link_libraries(${target} ${CHOREONOID_BASE_LIBRARIES})

Specifying libraries to link to the plugin. By using the CHOREONOID_BASE_LIBRARIES variable obtained by find_package, you can link the complete set of libraries that form the plugin's foundation. ::

 install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})

Settings for installing the built plugin file to Choreonoid's plugin directory. The installation destination can be specified with the CHOREONOID_PLUGIN_DIR variable like this.



Note that in CMake, when you specify a library defined in the same project with target_link_libraries, links to all libraries that library depends on are also performed. For example, since CnoidBase depends on Qt libraries, the above description will also link Qt libraries to the HelloWorld plugin. In this way, this method allows you to write concisely without worrying too much about the details of which libraries to link.