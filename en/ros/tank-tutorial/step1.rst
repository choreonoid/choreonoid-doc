Step 1: Control Tank by subscribing to Joy topic
================================================

Step 1 describes how to send robot commands to the robot controller through ROS-based communication, and control the robot based on the commands. Specifically, the status of the joystick is sent as a Joy topic, which is received by the controller so that the Tank robot can be controlled by the joystick.

.. contents::
   :local:

.. _ros_tank_tutorial_invoke_choreonoid_node:

Starting the Choreonoid node
----------------------------

.. highlight:: sh

After the preparations described in the previous section are completed, start Choreonoid.

As described in :doc:`../run-choreonoid` , the following command can be used to start the Choreonoid node in ROS. ::

 rosrun choreonoid_ros choreonoid

If the startup is successful, the main window of Choreonoid will be displayed.


Creating a simulation project
-----------------------------

Let's load models to be simulated on Choreonoid and build a project. In this tutorial, we will use a tank-like :ref:`tank_model` as the model corresponding to the robot. After starting Choreonoid, follow the steps below to do :doc:`../../simulation/simulation-project` for this model.

1. Create a world item
2. Load the Tank model as a child item of the world item.
3. Load an appropriate environment model as a child item of the world item.
4. Create an AIST simulator item as a child item of the world item.
5. Configure the properties of the AIST simulator ("Self Interference Detection" to true)

This process is basically the same as :doc:`../../simulation/tank-tutorial/step1` in the :doc:`../../simulation/tank-tutorial/index` , so please refer to that for further instructions.

There, we use a simple floor model as the environment model, but you can also use the plant model used in :ref:`tank_tutorial_use_labo_model` , which will give you a more realistic image when communicating camera images later in this tutorial. (However, the model will be heavy, so depending on your PC environment, a simple floor model may be easier to handle.) The repository for this tutorial on Github uses the plant model.

Once you have built your project, save it as a project file using "File" - "Save Project As" from the main menu. Create a subdirectory named "project" in the top directory of this tutorial, and store the file in the subdirectory with the name "step1.cnoid".

This will result in the following directory/file structure in the package for this tutorial.

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   - CMakeLists.txt
   - package.xml
   + project
     - step1.cnoid

To load the project again after the completion of Choreonoid, add the project file name as an option to the command used in :ref:`ros_tank_tutorial_invoke_choreonoid_node` . For example, execute ::

 roscd choreonoid_ros_tank_tutorial

to move to the tutorial directory, and by executing ::

 rosrun choreonoid_ros choreonoid project/step1.cnoid

you can start the Choreonoid node including the step1 project.

It is recommended to use this method to start choreonoid and load the project until :ref:`ros_tank_tutorial_introduce_launch_file` as described below.

Once the project has been built, start the simulation in the same way as in :ref:`tank-tutorial-step1-start-simulation` of :doc:`../../simulation/tank-tutorial/index` . As explained there, the turret part will fall down due to gravity, and the chassis will not move in any particular way. This is because the Tank just exists, and no controller has been introduced to control it, which is a natural result.

The goal of step 1 is to be able to freely control this Tank robot.

Preparing the gamepad
---------------------

This tutorial uses a joystick as an input device to freely control the Tank robot. There are many different types of joysticks, but for controlling this type of robot, the best choice is what is called a gamepad. You will need to prepare an appropriate gamepad for the tutorial, and most of the ones that connect via USB will work. However, using a gamepad that is compatible with the :ref:`ros_tank_tutorial_choreonoid_joy` , which will be explained later, will allow you to operate the robot smoothly. Please refer to the :ref:`simulation-tank-tutorial-gamepad` in the :doc:`../../simulation/tank-tutorial/index` for the supported gamepads.

Once the gamepad is prepared, connect it to the PC beforehand.

Sending gamepad status by joy node
----------------------------------

Since the theme of this tutorial is the use of ROS, we will use the ROS function to exchange the gamepad status as well. 
By doing so, various devices that support ROS can be used, and remote control can be performed by communication between remote hosts. In this section, we will first prepare to send the status of the gamepad.

Publishing joy topic
~~~~~~~~~~~~~~~~~~~~

In ROS, it is possible to define various data as "messages" and send them as "topics". Sending data of a topic is called "Publish" in ROS, and receiving a published topic is called "Subscribe". This is based on a software design model called the "Publish-Subscribe model," where data is published without specifying the recipient, and can be subscribed to from anywhere. ROS users may be familiar with this mechanism, but if not, please refer to the ROS documents on that.

In this tutorial, the state of the gamepad is published as a ROS topic and is subscribed from the robot controller.
To achieve this, you first need a program that publishes the gamepad status.
The program that performs such ROS communication is called a "ROS node". 
As a matter of fact, a ROS node called "Joy node" is available as a standard ROS package to publish the state of the gamepad (joystick), so let's try that first.

In the following, we will explain how to start the Joy node, as well as what the topics and messages of ROS are exactly.
If you are already familiar with the basics of ROS, you can skip the following and proceed to the next section, :ref:`ros_tank_tutorial_choreonoid_joy` .

Installing and starting joy node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

First, install the corresponding "joy package" in order to be able to use the joy node. ::

 sudo apt install ros-melodic-joy

This is the package name for Ubuntu 18.04 (Melodic); for Ubuntu 16.04 (Kinetic), you can install it with the following command. ::

 sudo apt install ros-kinetic-joy

If the Joy package has been successfully installed, you can start the joy node with the following command. ::

 rosrun joy joy_node

Before running this, however, make sure you have your joystick connected to your PC.
A joystick with a standard USB connection should work.

.. _ros_tank_tutorial_check_joy_topic:

Checking the joy topic
~~~~~~~~~~~~~~~~~~~~~~

When the Joy node starts and successfully detects the joystick, the topic that publishes the status of the joystick's axis and buttons will be generated. This can be checked with the following command. ::

 rostopic list

This command shows a list of topics currently available in the system. Check to see if ::

 /joy

is displayed here. This is the topic that the joy node is publishing to, and it is named "/joy".
Topic names can be managed hierarchically, similar to the file system, with the first slash indicating that the name is defined at the top level.

Let's see what this joy topic is all about. Try running the following command. ::

 rostopic info /joy

This will show the information for the topic corresponding to /joy. It should look something like this:

.. code-block:: none

 Type: sensor_msgs/Joy
 
 Publishers: 
  * /joy_node (http://hostname:34541/)
 
 Subscribers: None

The "sensor_msgs/Joy" shown here in the Type field is the message type for this topic.
This indicates what kind of data this topic will contain.
It also shows that the node that publishes this topic is /joy_node, and that there is no node that subscribes to it at this time.

Now let's check the contents of the message type. Please execute the following command. ::

 rosmsg show sensor_msgs/Joy

The following message will be displayed.

.. code-block:: none

 td_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  float32[] axes
  int32[] buttons

This represents the data structure of the message type "sensor_msgs/Joy".
Specifically, "axes" is a 32-bit floating-point number array that stores the positions of each axis of the joystick, and "buttons" is a 32-bit integer array that stores the state of each button (whether it is pressed or not).
In addition, the timestamp and ID value of this message are stored under "header".
The data structure is mapped to the corresponding types (such as std::vector<float> in C++) in each language in which ROS is coded, and can be accessed.

Let's check the actual contents of the published message. First, execute the following command. ::

 rostopic echo /joy

This is a command that will display the contents of the specified topic as text on the console. Execute this command and try manipulating the axis of the gamepad or pressing a button. This should produce output like the following.

.. code-block:: none

 header: 
   seq: 1
   stamp: 
     secs: 1585302374
     nsecs: 941266549
   frame_id: ''
 axes: [0.0, 0.03420161083340645, 0.0, 0.0, 0.0, 0.0]
 buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

The current value of each member is displayed in correspondence with the message type shown above. Here, for example, the second element of "buttons" is set to "1", indicating that the second button is pressed.

To quit this command, press Ctrl + C. If you do not see the above, then your gamepad may not be connected properly. In order to proceed with this tutorial, please make sure that this is working properly first.

.. _ros_tank_tutorial_choreonoid_joy:

Choreonoid version joy node
---------------------------

The Joy node introduced in the previous section can publish the state of the gamepad, but in this tutorial, we would like to use the "Choreonoid joy node" instead of the original joy node. This is supported by the "choreonoid_joy" package introduced in :ref:`ros_tank_tutorial_package_setup`, and can be started by the following command. ::

 rosrun choreonoid_joy node

This node is almost the same as the original joy node in terms of functionality, but it differs in that it standardizes the mapping of the axes and buttons of the gamepad.

As mentioned above, the state of the gamepad is stored in the Joy message, but the order in which the actual axes and buttons are arranged in the "axes" and "buttons" arrays differs depending on the model of the gamepad. This is because the order in which the hardware devices return the axes and buttons through the driver is different, but the original joy node simply stores the axes and buttons in the same order. However, this makes it difficult to use various models of gamepads in the same way. Although recent gamepad models have similar axes and buttons, the actual movements of the robot would be different even if the same axes and buttons are operated.

Therefore, Choreonoid joy node defines a standard order (mapping) for axes and buttons, and each actual gamepad model is converted to that mapping and stored in the Joy message. The Joy message subscriber can then handle various models of gamepads in the same way by simply reading the standard mappings.

For the tutorial, the sample program should be as simple as possible, and it should also be able to operate in the same way. Therefore, in this tutorial, we decided to use the Choreonoid joy node for publishing Joy topics. To proceed with the tutorial, please start the choreonoid_joy node with the above command. You can check the operation in the same way as with the standard Joy node.

Please note that although it is written in such a way that it can be used for various models, only the models listed in the :ref:`simulation-tank-tutorial-gamepad` are actually supported. For other models, Joy topics will be published, but the mapping will not be standardized, so the output will be the same as the ROS original joy node.

Building the controller
-----------------------

Now that the gamepad status is published, we would like to use this to implement a controller to enable the gamepad to control the Tank robot. What we will do below is essentially the same as the build process in :doc:`../../simulation/tank-tutorial/step2` of the :doc:`../../simulation/tank-tutorial/index` . However, in this tutorial, we need to build the controller in the ROS catkin environment and make it usable, so the specific build method and description will be different. This section presents how to build the controller as well as the source code of the controller.

.. _ros_tank_tutorial_step1_source:

Source code of the controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. highlight:: c++
   :linenothreshold: 7

Here is the source code of the controller. This controller inherits from SimpleController as well as the controller created in the :doc:`../../simulation/tank-tutorial/index` . SimpleController itself is independent of ROS, but by simply adding ROS code to it, you can take advantage of ROS functions. ::

 #include <cnoid/SimpleController>
 #include <cnoid/Joystick>
 #include <ros/node_handle.h>
 #include <sensor_msgs/Joy.h>
 #include <mutex>

 using namespace cnoid;

 class RttTankController : public SimpleController
 {
     std::unique_ptr<ros::NodeHandle> node;
     ros::Subscriber subscriber;
     sensor_msgs::Joy latestJoystickState;
     std::mutex joystickMutex;

     Link* trackL;
     Link* trackR;
     Link* turretJoint[2];
     double qref[2];
     double qprev[2];
     double dt;

 public:
     virtual bool configure(SimpleControllerConfig* config) override
     {
	 node.reset(new ros::NodeHandle);
	 return true;
     }

     virtual bool initialize(SimpleControllerIO* io) override
     {
	 std::ostream& os = io->os();
	 Body* body = io->body();
	 dt = io->timeStep();

	 trackL = body->link("TRACK_L");
	 trackR = body->link("TRACK_R");
	 io->enableOutput(trackL, JointVelocity);
	 io->enableOutput(trackR, JointVelocity);

	 turretJoint[0] = body->link("TURRET_Y");
	 turretJoint[1] = body->link("TURRET_P");
	 for(int i=0; i < 2; ++i){
	     Link* joint = turretJoint[i];
	     qref[i] = qprev[i] = joint->q();
	     joint->setActuationMode(JointTorque);
	     io->enableIO(joint);
	 }

	 subscriber = node->subscribe(
	     "joy", 1, &RttTankController::joystickCallback, this);

	 return true;
     }

     void joystickCallback(const sensor_msgs::Joy& msg)
     {
	 std::lock_guard<std::mutex> lock(joystickMutex);
	 latestJoystickState = msg;
     }

     virtual bool control() override
     {
	 sensor_msgs::Joy joystick;
	 {
	     std::lock_guard<std::mutex> lock(joystickMutex);
	     joystick = latestJoystickState;
	 }
	 joystick.axes.resize(Joystick::NUM_STD_AXES, 0.0f);
	 joystick.buttons.resize(Joystick::NUM_STD_BUTTONS, 0);

	 static const int trackAxisID[] =
	     { Joystick::L_STICK_H_AXIS, Joystick::L_STICK_V_AXIS };
	 static const int turretAxisID[] =
	     { Joystick::R_STICK_H_AXIS, Joystick::R_STICK_V_AXIS };

	 double pos[2];
	 for(int i=0; i < 2; ++i){
	     pos[i] = joystick.axes[trackAxisID[i]];
	     if(fabs(pos[i]) < 0.2){
		 pos[i] = 0.0;
	     }
	 }
	 // set the velocity of each tracks
	 trackL->dq_target() = -2.0 * pos[1] + pos[0];
	 trackR->dq_target() = -2.0 * pos[1] - pos[0];

	 static const double P = 200.0;
	 static const double D = 50.0;

	 for(int i=0; i < 2; ++i){
	     Link* joint = turretJoint[i];
	     double pos = joystick.axes[turretAxisID[i]];
	     if(fabs(pos) < 0.15){
		 pos = 0.0;
	     }
	     double q = joint->q();
	     double dq = (q - qprev[i]) / dt;
	     double dqref = 0.0;
	     double deltaq = 0.002 * pos;
	     qref[i] += deltaq;
	     dqref = deltaq / dt;
	     joint->u() = P * (qref[i] - q) + D * (dqref - dq);
	     qprev[i] = q;
	 }

	 return true;
     }

     virtual void stop() override
     {
	 subscriber.shutdown();
     }
 };

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttTankController)

Create a subdirectory named "src" in the package directory and save this source code there with the file name "RttTankController.cpp". Then, the file structure of the package will look like the following.

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   - CMakeLists.txt
   - package.xml
   + project
     - step1.cnoid
   + src
     - RttTankController.cpp

.. note:: The "Rtt" prefix in the controller class name and source file name is an abbreviation for "ROS Tank Tutorial". Some of the classes created in this tutorial are relatively generic, so you may find similar classes created or provided elsewhere. In order to distinguish them, this prefix is given to the classes and files created in this tutorial.

In the following, we will first explain how to build this source code and run it in a simulation, and then explain the contents of the source code.

Editing the CMakeLists.txt file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. highlight:: cmake

In :ref:`ros_tank_tutorial_edit_package_xml` , we explained that an XML file called package.xml is needed to build a Catkin package. Actually, there is another file called "CMakeLists.txt" that is needed to build the package. This is a file for the CMake build system, and is used when some build process is required, such as when the package contains C++ source code.

For more information about CMake and CMakeLists.txt, please refer to the manual of CMake. CMake is a very popular tool, and is originally used in both ROS and Choreonoid. This section assumes that the basic information is understood.

The template of CMakeLists.txt is automatically generated in :ref:`ros_tank_tutorial_make_package` , and is saved directly under the project directory. Edit the file so that it has the same contents as below. ::

 cmake_minimum_required(VERSION 3.5.0)
 project(choreonoid_ros_tank_tutorial)
 
 find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   image_transport
   choreonoid
   )
 
 catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)
 
 set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
 set(CMAKE_CXX_EXTENSIONS OFF)

 include_directories(${catkin_INCLUDE_DIRS})
 
 add_subdirectory(src)

The following is an explanation of this content. First of all, ::

 cmake_minimum_required(VERSION 3.5.0)

specifies that the version of CMake used to build the package must be 3.5.0 or higher.
Currently, the latest Choreonoid development version requires this minimum version due to the CMake commands used internally. The automatically generated CMakeLists.txt may contain a lower version than this, in which case the Choreonoid related packages cannot be built. If you have Ubuntu 16.04 or later, the official CMake package satisfies this requirement.

Next, ::

 project(choreonoid_ros_tank_tutorial)

sets the project name for this package. This should usually be the same as the package name. ::

 find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   image_transport
   choreonoid
   )

This function detects dependent packages. In this example, the following packages are set as dependencies.

* roscpp: C++ library for ROS
* std_msgs: Standard ROS messages
* sensor_msgs: Sensor related messages
* image_transport: Library for image transfer
* choreonoid: The main body of Choreonoid

The contents described here generally overlap with the dependent packages described in :ref:`ros_tank_tutorial_edit_package_xml` . However, the above packages are only the libraries required to build C++ programs,
and they are not necessarily the same as those described in package.xml. ::

 catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)

is to prevent Catkin from generating CMake Config and pkg-config files. These files are required to use the packages created here from other packages, and are mainly for libraries. Since such a package is not created in this tutorial, the generation process is not necessary. Also, as mentioned in :ref:`ros_tank_tutorial_edit_package_xml`,  the build type of the package should be "cmake", but it seems that the process of generating config files does not work well with that build type. For this reason, the above description is included in this package. ::

 set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
 set(CMAKE_CXX_EXTENSIONS OFF)

Here we set the C++ version to be used for compilation. 
Choreonoid is coded for C++11 or higher, including the public API of the library, and the programs that use the Choreonoid libraries must be built with the same or higher C++ version. However, some compilers may default to an older C++ version. Catkin doesn't seem to have any settings for this, so you will need to set the C++ version explicitly.

When the choreonoid package is specified in find_package, the version of C++ used in Choreonoid is set in the variable named CHOREONOID_CXX_STANDARD. Basically, the C++ version should be set to be the same as this.
In CMake, the version of C++ can be set by the variable CMAKE_CXX_STANDARD. Turning off CMAKE_CXX_EXTENSIONS disables the compiler's own extensions. In the case of GCC, if you don't include this description, the compiler's own extensions will be enabled, but for ease of maintenance, this description is included. It is possible to build without this description.

Note that GCC version 6 or higher uses C++14 by default, while Ubuntu 18.04 GCC is version 7, so Ubuntu 18.04 can be built without this description. On the other hand, the GCC installed on Ubuntu 16.04 is an older version and does not seem to be C++11 or higher by default, so without this description, a compile error will occur. ::

 include_directories(${catkin_INCLUDE_DIRS})

This description specifies additional include directories. The variable catkin_INCLUDE_DIRS is set to the include directories required when using the dependent package specified by find_package. This description allows you to use the header files included in those packages. If there are other libraries to be used, the corresponding include directories should be specified here. Note that the include directories of the libraries provided by Choreonoid do not necessarily need to be specified here. They will be automatically set when the CMake commands for building Choreonoid plugins, controllers, etc. are executed. ::

 add_subdirectory(src)

In this tutorial, the source files of the controllers written in C++ are stored separately in the "src" directory. In accordance with this structure, the CMake description directly corresponding to each source file is written in CMakeLists.txt in the src directory, and it is imported here.

Adding CMakeLists.txt to the src directory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For the above "CMakeLists.txt in src directory", create and add it with the following contents. ::

 choreonoid_add_simple_controller(RttTankController RttTankController.cpp)
 target_link_libraries(RttTankController ${roscpp_LIBRARIES})

choreonoid_add_simple_controller is a function that becomes available when choreonoid is detected by find_package. This function is used to build the binary of Choreonoid's simple controller, and can be used in the same way as CMake's built-in functions such as add_executable and add_library. In this example, the target name "RttTankController" is set and RttTankController.cpp is specified as the source file.

Also, the target_link_libraries function is used to specify the link to the dependent libraries. What is specified here is a link to the set of library files that constitute the roscpp library. This is set in the variable roscpp_LIBRARIES when roscpp is specified in find_package, so it is specified using this variable.

In addition, other Choreonoid libraries are also required to build a simple controller, but the links to the basic libraries such as CnoidUtil and CnoidBody are automatically specified by choreonoid_add_simple_controller. Therefore there is no need to specify them here. The include directories and compile options related to these libraries are also automatically set by choreonoid_add_simple_controller.

Building the controller
~~~~~~~~~~~~~~~~~~~~~~~

.. highlight:: sh

Once the source code of the controller and the CMakeLists.txt file are written, you are ready to build.
The build can be done with the following Cakin command. ::

 catkin build

This command can be run from any directory in the Catkin workspace.
Please refer to :ref:`ros_catkin_build_command` in :doc:`../build-choreonoid` for how to build.

It is recommended that you also :ref:`ros_catkin_config_cmake_build_type` when building. Usually, you should set the build type to "Release". This can be set with the following command ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

Setting this before building will enable optimizations in compilation and produce more efficient binaries.
Note that optimization will not be enabled unless this setting is set.

It is also possible to specify the default build type in the package by adding a description in CMakeLists.txt.
In this case, add the following description to the main CMakeLists.txt file.

.. code-block:: cmake

 if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release CACHE STRING
     "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
     FORCE)
 endif()

The appropriate place to add this is immediately after setting the project name with the project function.
With this description, Catkin will apply the optimized "Release" build without setting the CMake build type.

If you see the following output in the console after running catkin build, the build was successful.

.. code-block:: none

 ...
 Starting  >>> choreonoid_ros_tank_tutorial
 Finished  <<< choreonoid_ros_tank_tutorial                [ 3.0 seconds ]
 ...
 [build] Summary: All ? packages succeeded!                                  
 ...

If the build fails, compile error messages will be displayed, so please modify the source code or CMakeLists.txt according to the messages.

.. _ros_tank_tutorial_step1_introduce_controller:

Introducing the controller
--------------------------

After you have successfully built the controller, it is time to introduce it into your simulation project.

The introduction will be done in the same way as in :ref:`simulation-tank-tutorial-introduce-controller` of the :doc:`../../simulation/tank-tutorial/index` . Since the name of the controller created this time is "RttTankController", it is recommended to use the same name for the item. Also, for :ref:`simulation-tank-tutorial-set-controller` , select "RttTankController.so" which is generated by the above build process. This file should be generated in the standard controller directory, but if you don't see it, the build has failed, so check the previous steps.

The item tree should now look like the following.

.. code-block:: none

 + World
   + Tank
     - RttTankController
   - Labo1
   - AISTSimulator

For "Labo1", you can use Floor or any other environment model.

This completes the simulation project for Step 1. Overwrite the project file with this configuration.

Running the simulation and controlling the Tank robot with the gamepad
----------------------------------------------------------------------

Let's run the simulation.

If the :ref:`ros_tank_tutorial_choreonoid_joy` has been activated, you should be able to control the Tank robot with the connected gamepad. This is achieved by the controller subscribing to the status of the gamepad that the joy node publishes as a joy topic.

If it is a standard gamepad that is supported by Choreonoid, the left analog stick can be used to move the chassis (crawler) forward, backward, and turn left and right. The right analog stick can also be used to rotate the turret and barrel.

Checking the Joytopic connection status
---------------------------------------

.. highlight:: sh

While the simulation is running, let's check the connection status of the joy topic.

First, let's run the following command again, which we tried in :ref:`ros_tank_tutorial_check_joy_topic` . ::

 rostopic info /joy

The "Subscribers" field, which was "None" earlier, should now look like this.

.. code-block:: none

 Subscribers: 
  * /choreonoid (http://host:37373/)

You can see that /choreonoid has been added as a Subscriber. This represents a node that is subscribed to this topic. The actual object that is subscribing to this topic is RttTankController, but here it is shown as choreonoid. This is because ROS nodes are created in units of OS processes, and anything running in a Choreonoid process is a Choreonoid node. The simple controller also runs in the Choreonoid process, so it is a choreonoid node.

Next, let's visualize the connection status in a graph, since ROS has a tool called "rqt_graph" to do this. ::

 rosrun rqt_graph rqt_graph

You should see something like the following.

.. image:: images/step1-node-graph.png
    :scale: 70%

The actual display depends on the settings of rqt_graph; if you set the combo box in the upper left corner of rqt_graph and the checkboxes in the area below it to the same settings as shown above, you will see the same graph.

In any case, this graph shows that the joy topic published by the choreonoid_joy node is subscribed to by the choreonoid node, and there is a connection between the two nodes.

When ROS is used for control communication as shown in this example, we can not only communicate with each other but also cooperate with the ROS tools in this way. Many useful tools are available in ROS and being able to utilize them is a big advantage when introducing ROS.

.. _ros_tank_tutorial_introduce_launch_file:

Introducing a launch file
-------------------------

In Step 1, we have run the following ROS nodes so far.

* choreonoid itself (the project of step1.cnoid)
* choreonoid_joy
* rqt_graph

We have invoked each of them by entering the corresponding command from the terminal, but it is tedious to enter three commands when we want to do the same thing again, and we don't know if we can remember each one.
The roslaunch command, which is provided in ROS, allows us to perform these operations at once.

.. highlight:: xml

Which node to launch and how to launch it is described in an XML file called the launch file. To launch the three nodes in this case, create the following launch file. ::

 <launch>
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
         args="$(find choreonoid_ros_tank_tutorial)/project/step1.cnoid --start-simulation" />
   <node pkg="rqt_graph" name="rqt_graph" type="rqt_graph" />
 </launch>

Please refer to the ROS manual for more information about the lauch file. Basically, you write as many "node" tags as you need to start ROS nodes in the "launch" tag. In this example, the following processes are performed for each. ::

 <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />

Execute the node command to invoke the choreonoid_joy node of the choreonoid_joy package. ::

 <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
       args="$(find choreonoid_ros_tank_tutorial)/project/step1.cnoid --start-simulation" />

Execute the choreonoid command to start the choreonoid node of the choreonoid_ros package. This will start the Choreonodi itself.

The following "args" are the arguments given to the choreonoid command. First, the project file is specified as an argument. ::

 $(find choreonoid_ros_tank_tutorial)

This expression returns the directory of the choreonoid_ros_tank_tutorial package.
It specifies the project file named step1.cnoid that exists in the project directory. Also, the ::

 --start-simulation

argument is an option to automatically start the simulation after the project is loaded.
If this option is specified, the simulation will be started just by executing the launch file.

Finally, add ::

 <node pkg="rqt_graph" name="rqt_graph" type="rqt_graph" />

to run rqt_graph as well.

.. highlight:: sh

This launch file should be saved in the "launch" directory of the choreonoid_ros_tank_tutorial package. Then the package for this tutorial will have the following file structure.

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   - CMakeLists.txt
   - package.xml
   + launch
     - step1.launch
   + project
     - step1.cnoid
   + src
     - CMakeLists.txt
     - RttTankController.cpp

Once this is done, you can execute this launch file by entering the following command from the terminal. ::

 roslaunch choreonoid_ros_tank_tutorial step1.launch

By executing the launch file in this way, you will be able to do what you have done in step 1 again.
In ROS, many nodes are often combined to build a system, and in such cases, roslaunch is indispensable.

When you want to quit running roslaunch, you can type Ctrl + C on the terminal where roslaunch is running. This will terminate the execution of all nodes launched by roslaunch.

Source code description
-----------------------

Finally, let's look at the :ref:`ros_tank_tutorial_step1_source` .

The joint control part of this controller is almost the same as the one created in the following steps of :doc:`../../simulation/tank-tutorial/index` .

* :doc:`../../simulation/tank-tutorial/step2` ( :ref:`tank_tutorial_step2_implementation` )
* :doc:`../../simulation/tank-tutorial/step3` ( :ref:`simulation-tank-tutorial-step3-implementation` )

The difference is that in this controller, the command value of the control is obtained by subscribing to the Joy topic, so we will focus on that part below.

.. highlight:: c++

First of all, the following headers of roscpp are included. ::

 #include <ros/node_handle.h>
 #include <sensor_msgs/Joy.h>

By including <ros/node_handle.h>, you will be able to use the NodeHandle class of roscpp. This corresponds to a ROS node, and allows you to publish and subscribe to topics via objects of this class.

In addition, <sensor_msgs/Joy.h> is a header corresponding to the Joy message. By including it, you can access Joy messages in C++. ::

 #include <mutex>

Enables the use of the mutex class from the standard C++ library. The topic communication is asynchronous, and you need to control the exclusivity of the state retrieved from it when passing it to the control loop. A mutex is required to do this.

Let's look at the variables involved in subscribing to a joy topic. First, ::

 std::unique_ptr<ros::NodeHandle> node;

is a variable that corresponds to a ROS node. To be precise, a ROS node is assigned to each process, and this is a handle of the node, which can be created and used multiple times in a process. Here, it is managed as a pointer using std::unique_ptr, and the actual creation of the object is done in the initialization function described below. ::

 ros::Subscriber subscriber;

In order to subscribe to a topic, you need to create a subscriber. This variable is used to store the created subscriber. ::

 sensor_msgs::Joy latestJoystickState;

This is the variable to store the Joy type message. This type is defined in <sensor_msgs/Joy.h>. ::

 std::mutex joystickMutex;

This is a mutex for exclusivity control in Joy message exchange.

The ROS NodeHandle is created in the following function. ::

 virtual bool configure(SimpleControllerConfig* config) override
 {
     node.reset(new ros::NodeHandle);
     return true;
 }

The NodeHandle created here needs to be deleted when finished using it.
To do this automatically, a smart pointer with std::unique_ptr is used.

The configure function implemented here is one of the initialization functions defined in the SimpleController class ( :ref:`simulation-implement-controller-simple-controller-class-supplement` ). It is defined as a virtual function, and by overriding it, the initialization process can be implemented. In fact, SimpleController has three virtual functions for initialization, each of which will be called at the following times.

* configure: Called when the controller is introduced to the project.
* initialize: called just before the simulation starts
* start: Called when the controller is about to start running after the simulation initialization is complete.

Normally, initialization is done in the initialize function, but since it is done only when the simulation starts, any initialization that needs to be done before the simulation starts needs to be written in the configure function. In the case of ROS, the connection between nodes is very important, and we want to make sure that it is all done before the simulation starts. In order to achieve this, the NodeHandle needs to be created before the simulation starts, so we use the configure function to do that.

The normal initialization process is implemented in the initialize function.
The bulk of the implementation is the preparation for the crawler and the control of the turret and barrel axes, which is explained in detail in :doc:`../../simulation/tank-tutorial/index` , so we will not go into details here.
As for the part related to ROS, the following process is described. ::

 subscriber = node->subscribe(
     "joy", 1, &RttTankController::joystickCallback, this);

This creates the subscriber for the joy topicr by specifying the target topic name in the NodeHandle's subscribe function.
The generated subscriber is stored in a variable of the Subscriber type.
This is a reference to the actual subscriber object, which is used to manage the lifetime of the subscriber.

The second argument specifies the size of the queue to be used for receiving topics. By increasing this value, it is possible to reduce the number of messages to be received. However, in this sample, it is enough to get the latest joystick status, so we don't care about missing messages during the process, and specify 1 as the queue size.

The third and fourth arguments specify the callback function to be used when subscribing. There are several ways to specify the callback function, but here we are using the one that targets a member function and specifying the joystickCallbak function of RttTankController.

With the above description, when a joy topic is published, it will be received by the ROS node of Choreonoid and the received Joy message will be passed to the joystickCallback function. This receiving process is done asynchronously and the callback function will be called from a different thread than the controller control function, so be careful about that.

The callback function is implemented as follows. ::

 void joystickCallback(const sensor_msgs::Joy& msg)
 {
     std::lock_guard<std::mutex> lock(joystickMutex);
     latestJoystickState = msg;
 }

The argument of the callback function is the message type of the target topic. Here, a message of the sensor_msgs::Joy type is passed as the argument.

What we want to do here is to pass the content of this message (the state of the gamepad) to the control code of the simple controller. For this purpose, we use a variable called "latestJoystickState" of the same message type, and copy the contents of the received message to this variable. By referring to this variable in the control function as well, the state of the gamepad is reflected in the control.

Note that this callback function can be called at any time from a different thread than the controller's control function as mentioned above, In such a case, there is a possibility that the overwriting of the latestJoystickState by this function and the reference to the same variable by the control function will conflict in terms of timing. To avoid this, it is necessary to apply exclusion control to the variable access. This is achieved with joystickMutex.

The part of the control function that refers to this variable is as follows. ::

 virtual bool control() override
 {
     sensor_msgs::Joy joystick;
     {
         std::lock_guard<std::mutex> lock(joystickMutex);
         joystick = latestJoystickState;
     }
     joystick.axes.resize(Joystick::NUM_STD_AXES, 0.0f);
     joystick.buttons.resize(Joystick::NUM_STD_BUTTONS, 0);
     ....

Here, we are preparing a variable "joystick" of the same Joy type and trying to copy the contents of the latestJoystickState to that variable. In this part of the code, the exclusive-control using joystickMutex is also applied to avoid conflicts with the callback function regarding the latestJoystickState variable.

In order to minimize the scope of the exclusion control, we purposely introduced a variable named joystick so that we only need to apply exclusion control to the copy to this variable. In this sample, the implementation of the control function is very simple and does not take a long time to execute, so there is no particular problem even if you apply the exclusive control to the entire control function and refer directly to the latestJoystickState. However, if the control becomes more complex and takes more time to execute, it is desirable to reduce the scope (time) of the exclusive control as much as possible, as shown in this sample.

Note that the code part ::

 joystick.axes.resize(Joystick::NUM_STD_AXES, 0.0f);
 joystick.buttons.resize(Joystick::NUM_STD_BUTTONS, 0);

is not necessary if you are using the choreonoid_joy node, but if you are using the ROS standard joy node, the number of axes and buttons will change depending on the joystick to be connected, so this code is included just in case.

After that, the controller gets the current status of the gamepad from the joystick variable copied here, and calculates the command value from it, and uses it to command the drive speed of the crawler and perform the PD control of the turret and barrel axes. The specific process of the control is the same as those described in the :doc:`../../simulation/tank-tutorial/index` , so the details are omitted here.

Finally, the process when the controller is stopped is implemented in the following function. ::

 virtual void stop() override
 {
     subscriber.shutdown();
 }

Once the controller is stopped, there is no need to subscribe to the joy topic anymore, so the subscriber's shutdown function is executed to terminate the subscription process.

Tip: Make sure you are running in a ROS environment.
----------------------------------------------------

The above controller code assumes that ROS is available and Choreonoid is running as a ROS node. However, it may happen that this controller is accidentally loaded in a non-ROS environment or in a normal Choreonoid process that is not a ROS node. In such a case, roscpp is not initialized, so it will crash at ::

 node.reset(new ros::NodeHandle);

when the configure function is executed.

To avoid this and make the controller binary safer, we can add a code to check the state of ROS to the configure function as follows ::

 virtual bool configure(SimpleControllerConfig* config) override
 {
     if(!ros::isInitialized()){
         config->os() << config->controllerName()
                      << "cannot be configured because ROS is not initialized." << std::endl;
         return false;
     }
     node.reset(new ros::NodeHandle);
     return true;
 }

The ros::isInitialized function will return true if the roscpp is correctly initialized in the ROS environment, otherwise it will return false. Therefore, if this function returns false, output an appropriate message and exit the configure function with false. In this case, the rest of the controller will not be executed, so a crash can be avoided and the user will know for sure that the controller cannot be used.

However, using a controller developed for ROS in a non-ROS environment may not be expected, so if you do not intend to disclose the controller to the public, you may not have to go this far. In order to keep the code simple and the explanation concise, this tutorial does not include such processing in the sample code.
