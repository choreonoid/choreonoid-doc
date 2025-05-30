Building Controllers
====================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents:: Table of Contents
   :local:

How to Build Controllers
------------------------

In :doc:`howto-implement-controller`, we explained the overview of controller implementation for Simple Controllers, focusing on source code description. Since the controller used there was one of Choreonoid's samples, it became available when you built or installed Choreonoid itself. However, when users develop new controllers, simply writing the controller source code is not enough to make it usable. Since Simple Controllers are written in C++, building (compiling, linking, etc.) is required to generate usable binary files from the source code. Here we explain the overview of building methods for Simple Controllers.

There are two main methods for implementing and building your own controllers:

1. Build together with Choreonoid itself
2. Build separately from Choreonoid itself

We will explain each method separately below.

.. _simulation-build-controller-method1:

Building Together with Choreonoid
---------------------------------

.. highlight:: cmake

In this method, similar to sample controllers, your own controllers are built together when building Choreonoid itself. In other words, when building your own controllers, you use the same commands to build Choreonoid, building the controllers as part of Choreonoid. If you are using Choreonoid built from source code, this method is likely the most convenient.

Choreonoid itself uses CMake for build descriptions, and this method includes your own controller's build description within it. In CMake, build descriptions are usually written in files named "CMakeLists.txt", so this method also describes the controller build in this file. You can concisely describe the Simple Controller build using the CMake function "choreonoid_add_simple_controller" defined in Choreonoid itself. The arguments for this function are as follows: ::

 choreonoid_add_simple_controller(controller_name source_file ...)

You can specify one or multiple source files.

You then need to make Choreonoid's build recognize this CMakeLists.txt. To do this, use the directory named "ext" included in Choreonoid's source directory. If there is a CMakeLists.txt in a subdirectory created under this directory, it will be recognized and incorporated into Choreonoid's build. For controllers as well, create a directory with an appropriate name under ext and store the corresponding source files and CMakeLists.txt there.

As a specific example, let's say you write C++ source code for your own "MyController" controller in a file called "MyController.cpp". First, create a subdirectory within Choreonoid's ext directory to store this file. Any name is fine, but here we'll create a directory called "MyController" to correspond with the controller name.

Then, create a CMakeLists.txt with the following content and save it in the same directory: ::

 choreonoid_add_simple_controller(MyController MyController.cpp)

.. highlight:: text

As a result, the directory/file structure will be as follows: ::

 Choreonoid source directory
  + ext
    + MyController
      - CMakeLists.txt
      - MyController.cpp

After this, building Choreonoid itself will also build MyController. That is, execute cmake and make in Choreonoid's build directory. If the build succeeds, a file called MyController.so (on Linux; MyController.dll on Windows) should be generated in the Simple Controller standard directory mentioned in :ref:`simulation-set-controller-to-controller-item`.

.. note:: Avoid running cmake directly on the controller's CMakeLists.txt created under ext. In this method, the controller's CMakeLists.txt is incorporated as part of Choreonoid's build description, so cmake cannot be applied to it independently.

.. note:: You can also incorporate source directories placed outside of ext. In that case, set the path to the source directory you want to incorporate in **ADDITIONAL_EXT_DIRECTORIES** in Choreonoid's CMake. You can set multiple paths by separating them with semicolons.

.. note:: When the controller configuration becomes complex, such as when the controller links to external libraries, you may need to write more in CMakeLists.txt besides choreonoid_add_simple_controller. In that case, refer to the CMake manual and the definition of the choreonoid_add_simple_controller function to write appropriate descriptions. (The choreonoid_add_simple_controller function is defined in src/Body/ChoreonoidBodyBuildFunctions.cmake in the Choreonoid source.)

Building Separately from Choreonoid
------------------------------------

This method assumes that Choreonoid is installed on the system and builds controllers to use with it separately.

"Installation" here means copying the files necessary for execution to specified system directories after building Choreonoid from source files, and setting paths to executable and library files. For installation methods, see:

* :ref:`build-ubuntu_install` in :doc:`../install/build-ubuntu`
* :ref:`build-windows-install` in :doc:`../install/build-windows`

Setting Compile Options
~~~~~~~~~~~~~~~~~~~~~~~

If Choreonoid is installed, build by setting the corresponding include and library paths. You also need to specify several other compile options.

For example, if Choreonoid is installed under /usr/local and you're compiling with gcc (g++), you would specify the following gcc compile options. (Replace /usr/local with the actual installation directory.)

* **-std=c++11** (Enable C++11)
* **-fPIC** (Compile for shared library)
* **-I/usr/local/include** (Add include path)

Similarly, link options would be:

* **--shared** (Link as shared library)
* **-L/usr/local/lib** (Add link path)
* **-lCnoidUtil -lCnoidBody** (Link Choreonoid's Util and Body libraries)

The Util and Body libraries are part of the libraries that make up Choreonoid. The Util library is a utility library that combines various functions, and the Body library is a library that combines functions related to :doc:`../handling-models/bodymodel`. Since Simple Controllers use the functions of these libraries, linking to at least these libraries is necessary.

.. note:: "/usr/local/include" and "/usr/local/lib" may be included in the compiler's include and library paths by default. In that case, the above "-I/usr/local/include" and "-L/usr/local/lib" are not necessary. However, if Choreonoid is installed in a directory other than "/usr/local", adding the corresponding paths is usually necessary.

Installing Controllers
~~~~~~~~~~~~~~~~~~~~~~

The generated controller binary file is usually copied (installed) to the controller standard directory. The standard directory is:

* /usr/local/lib/choreonoid-x.x/simplecontroller (x.x corresponds to the version number)

.. note:: The controller standard directory is provided because it's clear to keep files together there and easy to access from Simple Controller items. If you have reasons to store controllers in other directories, there's no particular problem doing so.

Using pkg-config
~~~~~~~~~~~~~~~~

.. highlight:: sh

When Choreonoid is installed, you can set compile options using `pkg-config <https://www.freedesktop.org/wiki/Software/pkg-config/>`_.

Specifically, executing ::

 pkg-config --cflags choreonoid-body

outputs the options necessary for compiling programs using the Body library, and executing ::

 pkg-config --libs choreonoid-body

outputs the options necessary for linking programs using the Body library.

By using these commands, you can build programs using Choreonoid without worrying much about where Choreonoid is installed or which libraries need to be linked.

The "choreonoid-body" specified in the command is the identifier corresponding to Choreonoid's Body library in pkg-config. When Choreonoid is installed, you can obtain information about each Choreonoid library with the following identifiers:

* **choreonoid-util** : Util library
* **choreonoid-body** : Body library
* **choreonoid-base** : Base library
* **choreonoid-body-plugin** : Body plugin library

When building Simple Controllers, you usually just need to use choreonoid-body.

.. note:: The Base library is a foundation library used when developing Choreonoid plugins. The Body plugin library makes Body plugin functionality available as a library from outside and is used when developing other plugins that depend on the Body plugin.

Note that for pkg-config to use the above identifiers, Choreonoid's installation location must be recognized by the pkg-config system. If you install in the default location "/usr/local", pkg-config will recognize it as is, but if you install Choreonoid in other directories, you may need to set environment variables like "PKG_CONFIG_PATH".

For example, if you installed Choreonoid under usr in your home directory, execute ::

 export PKG_CONFIG_PATH=$HOME/usr/lib/pkgconfig

.. _simulation-build-controller-commands:

Build Command Examples
~~~~~~~~~~~~~~~~~~~~~~

Here are examples of actual build commands for Ubuntu Linux.

Assume the controller source file is "MyController.cpp". Store this in an appropriate directory and navigate to that directory from the command line.

You can compile with the following command: ::

 g++ -std=c++11 -fPIC `pkg-config --cflags choreonoid-body` -c MyController.cpp

This compiles MyController.cpp and generates an object file called MyController.o.

Next, link with the following command: ::

 g++ --shared -std=c++11 -o MyController.so MyController.o `pkg-config --libs choreonoid-body`

This generates a file called MyController.so. This is the controller binary file that can be specified in the Simple Controller item's "Controller module" and used.

If necessary, also install to the standard directory: ::

 cp MyController.so `pkg-config --variable=simplecontrollerdir choreonoid-body`

This pkg-config usage obtains the path to the Simple Controller standard directory. If installed under /usr/local, add sudo to the above command: ::

 sudo cp MyController.so `pkg-config --variable=simplecontrollerdir choreonoid-body`

and execute.

.. note:: As with :ref:`simulation-build-controller-method1`, when the controller configuration becomes complex, such as when the controller consists of multiple source files or links to libraries other than CnoidBody, the above commands alone may not be sufficient for building. Handling such cases goes beyond the scope of this explanation and becomes a topic of general program development methods, so we'll omit it here.

Makefile Example
~~~~~~~~~~~~~~~~

.. highlight:: makefile
   :linenothreshold: 5

It's tedious to execute the commands mentioned above every time. To avoid this and simplify build operations, you can use the Make command. With the Make command, you describe the build method in a file named Makefile. Here's an example Makefile for building MyController: ::

 CONTROLLER=MyController.so
 SRC=MyController.cpp
 OBJ=$(SRC:%.cpp=%.o)
 
 $(CONTROLLER): $(OBJ)
	g++ --shared -std=c++11 -o $(CONTROLLER) $(OBJ) `pkg-config --libs choreonoid-body`
 
 %.o: %.cpp
	g++ -std=c++11 -fPIC `pkg-config --cflags choreonoid-body` -c $<
 
 install: $(CONTROLLER)
 	install -s $(CONTROLLER) `pkg-config --variable=simplecontrollerdir choreonoid-body`
 clean:
	rm -f *.o *.so

Due to Makefile specifications, lines 6, 9, 12, and 14 must be indented from the beginning of the line using tabs. (Spaces will cause errors.)

.. highlight:: sh

Create a file with the above content named "Makefile" in the directory containing MyController.cpp. Navigate to that directory from the command line and enter ::

 make

to build the controller. Then execute ::

 make install

to install the controller to the standard directory. (For make install, add sudo if necessary.)

This achieves the same result as executing the commands introduced in :ref:`simulation-build-controller-commands`.

For information on writing Makefiles, refer to the `Make manual <https://www.gnu.org/software/make/manual/>`_.

.. note:: Although omitted here, you usually also add options like -O2 or -O3 for compilation and linking. These are optimization options that make the generated program run faster. Alternatively, when debugging, add debugging options like -g to generate debugging information. For details on these, refer to compiler manuals and various information about C/C++ program development.

In practice, directly writing Makefiles is rare. It's more common to use build tools that allow higher-level descriptions, such as CMake. Since CMake is used for building Choreonoid itself, it's also used for controllers in :ref:`simulation-build-controller-method1`, but CMake can also be used when building controllers separately from Choreonoid. However, the CMake execution method and CMakeLists.txt description in that case differ slightly from :ref:`simulation-build-controller-method1`, so please be aware. For CMake, refer to the `CMake manual <https://cmake.org/documentation/>`_ separately.