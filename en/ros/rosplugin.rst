ROS Plugin
==========

.. contents::
   :local:

.. highlight:: sh

What is the ROS Plugin?
-----------------------

The ROS plugin is a plugin for Choreonoid included in the choreonoid_ros package.

The basic role of this plugin is to enable the use of "roscpp", a library for using ROS functionality from C++, within Choreonoid. The build environment for this is provided by the choreonoid_ros package, and roscpp initialization is performed first within the ROS plugin.

This mechanism enables various capabilities within Choreonoid, such as controlling robots through controllers that communicate via ROS, and visualizing or editing data from ROS. These capabilities can be implemented in Choreonoid framework components like views and items, or simulation components like controllers and sub-simulators.

The primary way to achieve this is for users to code the necessary functionality themselves as C++ programs. roscpp is a well-designed library with comprehensive documentation, making it relatively straightforward to implement software that uses ROS functionality through roscpp. If you're familiar with ROS and C++, consider this approach to achieve your goals.

Additionally, the ROS plugin provides functionality to publish sensor information from robots during simulation as ROS topics.
It's also possible to control robots using ros_control.
Using these features, you can control robots with minimal coding requirements.

Loading the ROS Plugin
----------------------

The ROS plugin is included in the choreonoid_ros package. To use the ROS plugin, you need to start Choreonoid as a ROS node provided by choreonoid_ros. We call this a Choreonoid node.

As described in :ref:`choreonoid_ros_run_choreonoid_node`, you can start a Choreonoid node using the rosrun command like other ROS nodes: ::

 rosrun choreonoid_ros choreonoid

Of course, you can also use other methods to start ROS nodes, such as roslaunch.

When Choreonoid is started as a ROS node this way, the ROS plugin is loaded and becomes available. In this case, the following message appears in the started Choreonoid's message view: ::

 ROS-plugin has been activated.

Please note that if this message doesn't appear, the ROS plugin hasn't been loaded and you cannot use ROS functionality within Choreonoid. (This happens when you start Choreonoid installed independently of ROS using the normal method.)

When starting a Choreonoid node, you might see the following in the message view:

.. code-block:: none

 Warning: The ROS master is not found.

In this case, the ROS master isn't running, and you cannot use ROS functionality.

Please first perform :ref:`choreonoid_ros_run_ros_master`.

roscpp Initialization by the ROS Plugin
---------------------------------------

When the ROS plugin is loaded, processing equivalent to the following code is executed in the plugin's initialization function:

.. code-block:: c++

 ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
 auto spinner = new ros::AsyncSpinner(0);
 spinner->start();

First, roscpp is initialized with the ros::init function. The node name is set to "choreonoid" by default. ROS-related command line options provided when starting the Choreonoid node are stored in argc and argv and passed to this initialization function. This handles remapping of node names and topic names.

After initialization, ros::AsyncSpinner is created and background processing of the ROS callback queue begins. This causes processing of Subscribers and other components created within the Choreonoid node to be performed in background threads. The main loop for processing GUI and other functions runs normally on Choreonoid's main thread, with ROS processing occurring in parallel.

Through this initialization process, you can freely create and use Publishers, Subscribers, and other components on Choreonoid using ros::NodeHandle and similar tools. Conversely, since the ROS plugin handles roscpp initialization, other modules running on Choreonoid must not execute initialization functions.

Also, as described above, callback queue processing occurs in a thread separate from the main thread, so each callback function also executes in a thread different from the main thread. Keep this in mind when implementing callback functions and include appropriate synchronization mechanisms as needed.

For specific usage of ros::NodeHandle and related components, see :doc:`tank-tutorial/index`.