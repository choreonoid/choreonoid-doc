ROS Plugin
==========

.. contents::
   :local:

.. highlight:: sh

What is the ROS plugin?
-----------------------

The ROS plugin is a plugin for Choreonoid included in the choreonoid_ros package.

The basic role of this plugin is to make "roscpp", which is a library to use ROS functions from C++, available on Choreonoid.
The build environment for this is provided by the choreonoid_ros package, and the roscpp is initialized first in the ROS plugin.

This mechanism makes it possible to control a robot by a controller that communicates with ROS, and to visualize and edit data on ROS in views and items that are components of the Choreonoid framework, and in controllers and sub-simulators that are components of the simulation function, on Choreonoid.

The first way to achieve this is for users to write the necessary functions as C++ program code by themselves.
Since roscpp is a well-designed library with extensive documentation, it is not very difficult to implement software that uses the functions of ROS using roscpp. If you are familiar with ROS and C++, you should consider using this means to achieve your goals.

On the other hand, it would be useful to have a default function to use, visualize, and edit frequently used ROS messages and services for robot control. This would make it possible to use such ROS functions without coding.
Since Choreonoid can implement such functions in the form of items and views, We are planning to implement such items and views.

However, the current ROS plugin generally has only the functions for the former, and you need to do your own coding for ROS integration. As a matter of fact, the ROS plugin included in `choreonoid_ros_pkg <https://github.com/fkanehiro/choreonoid_ros_pkg>`_ , which was previously developed at AIST, has functions that can be used in robot simulations as a standard feature (see the `manual <https://github.com/fkanehiro/choreonoid_ros_pkg_doc>`_ ) , and the latter can be realized to some extent. On the other hand, the current official ROS plugin have not yet been able to migrate such functions. This is something we hope to improve in the future, and if you would like to help or support us in this matter, please feel free to contact us on the `forum <https://discource.choreonoid.org>`_.

Loading the ROS plugin
-----------------------

The ROS plugin is included in the chorenoid_ros package.
To use it, you need to start Choreonoid as a ROS node provided by choreonoid_ros. This is called a Choreonoid node.

As described in :ref:`choreonoid_ros_run_choreonoid_node`, the Choreonoid node can be invoked using the rosrun command like any other ROS node: ::

 rosrun choreonoid_ros choreonoid

Of course, other methods such as roslaunch can also be used to start a ROS node.

Choreonoid invoked as a ROS node in this way has the ROS plugin loaded and ready to use.
In this case, the message view of the invoked Choreonoid will contain the following message: ::

 ROS-plugin has been activated.

Note that if this message is not output, the ROS plugin is not loaded and the ROS functions cannot be used on Choreonoid. (This is the case when Choreonoid, which is installed independently from ROS, is started in the usual way.

In addition, when a Choreonoid node is started, the following message may be displayed in the message view:

.. code-block:: none

 Warning: The ROS master is not found.

In this case, the ROS master has not been started, and the ROS functions cannot be used, and you should first run :ref:`choreonoid_ros_run_ros_master`.

Initializing roscpp with the ROS plugin
---------------------------------------

When the ROS plugin is loaded, the following code is executed in the initialization function of the plugin.

.. code-block:: c++

 ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
 auto spinner = new ros::AsyncSpinner(0);
 spinner->start();

First, roscpp is initialized with the ros::init function.
The node name is set to be "choreonoid" by default. Also, the ROS-related command line options given at the startup of the Choreonoid node are stored in argc and argv, which are passed to this initialization function. This also handles the remapping of node names and topic names.

After the initialization is completed, ros::AsyncSpinner is created and the background processing of the ROS callback queue starts. As a result, the processing of Subscriber, etc., created in the Choreonoid node is performed in the background thread.
On the main thread of Choreonoid, the main loop for processing GUI and other functions runs as usual, and ROS processing is performed in parallel with it.

With the above initialization process, we can freely create and use Publisher, Subscriber, etc. on Choreonoid using ros::NodeHandle. On the other hand, the ROS plugin is responsible for the initialization of roscpp, so you must not execute any initialization functions in other modules running on Choreonoid.

In addition, since the callback queue is processed in a different thread from the main thread as described above, the thread in which each callback function is executed is also different from the main thread. Please keep this in mind when implementing callback functions, and use exclusion control if necessary.

The specific usage of ros::NodeHandle, etc. is explained in :doc:`tank-tutorial/index`.
