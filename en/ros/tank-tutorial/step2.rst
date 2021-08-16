Step 2: Publish the JointState topic to output the state of Tank
================================================================

In Step 2, we will explain how to output the robot's state externally by communicating using ROS, specifically, by publishing the ROS topic.

.. contents::
   :local:

Overview
--------

In Step 1, we explained how the controller can subscribe to and receive ROS topics that are being published externally.
In step 2, we will learn the opposite process: how to publish and send a ROS topic from the controller side.

Specifically, we will enable publishing the state of each joint of the Tank as a JointState topic, and the published joint states of the robot can be subscribed to and used by other ROS nodes.

Controller for state output
---------------------------

.. highlight:: c++
   :linenothreshold: 15

In Choreonoid, the state output of the robot is basically implemented in the controller. Therefore, in this step, we will create a new controller called "RttJointStatePublisher" to publish the state of the joints. The source code is shown below. We will use this sample to explain how to publish the robot's state as a ROS topic. ::

 #include <cnoid/SimpleController>
 #include <ros/node_handle.h>
 #include <sensor_msgs/JointState.h>
 
 using namespace std;
 using namespace cnoid;
 
 class RttJointStatePublisher : public SimpleController
 {
     BodyPtr ioBody;
     ros::NodeHandle node;
     ros::Publisher jointStatePublisher;
     sensor_msgs::JointState jointState;
     double time;
     double timeStep;
     double cycleTime;
     double timeCounter;
 
 public:
     virtual bool configure(SimpleControllerConfig* config) override
     {
         node.reset(new ros::NodeHandle(config->body()->name()));
         jointStatePublisher = node->advertise<sensor_msgs::JointState>("joint_state", 1);
         return true;
     }
         
     virtual bool initialize(SimpleControllerIO* io) override
     {
         ioBody = io->body();
 
         int n = ioBody->numJoints();
         jointState.name.resize(n);
         jointState.position.resize(n);
         jointState.velocity.resize(n);
         jointState.effort.resize(n);
 
         for(int i=0; i < n; ++i) {
             auto joint = ioBody->joint(i);
             io->enableInput(joint, JointDisplacement | JointVelocity | JointEffort);
             jointState.name[i] = joint->name();
         }
 
         time = 0.0;
         timeStep = io->timeStep();
         double frequency = 50.0;
         cycleTime = 1.0 / frequency;
         timeCounter = 0.0;
 
         return true;
     }

     virtual bool control() override
     {
         time += timeStep;
         timeCounter += timeStep;
 
         if(timeCounter >= cycleTime) {
             
             jointState.header.stamp.fromSec(time);
 
             for(int i=0; i < ioBody->numJoints(); ++i) {
                 auto joint = ioBody->joint(i);
                 jointState.position[i] = joint->q();
                 jointState.velocity[i] = joint->dq();
                 jointState.effort[i] = joint->u();
             }
             
             jointStatePublisher.publish(jointState);
 
             timeCounter -= cycleTime;
         }
 
         return true;
     }
 };
 
 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttJointStatePublisher)

Basic structure of the controller for state output
--------------------------------------------------

The controller will be implemented as SimpleController as in step 1.

The basic structure of the controller is as follows. First, ::

 #include <cnoid/SimpleController>.

to include the definition of the underlying SimpleController class, and then define the target controller as a class that inherits from SimpleController. ::

 class RttJointStatePublisher : public SimpleController

Then, among the virtual functions defined in SimpleController, override ::

 virtual bool configure(SimpleControllerConfig* config)

to implement the process of creating the ROS publisher as the initialization process when introducing the controller.
Then ::

 virtual bool initialize(SimpleControllerIO* io)

to initialize the controller at the start of simulation. Then ::

 virtual bool control()

to describe the process of state output using a publisher.

This structure is almost the same as the RttTankController created in Step 1, but the use of the "control" function may be slightly different. The "control" function usually used to implement the control process, and in RttTankController, it the control of the Tank robot is implemented. However, the essence of this function is to perform some kind of processing repeatedly in the control cycle of the controller, and the content of the cycle processing does not necessarily need to be the control of the robot. Therefore, this function can also be used to output the robot's status, as shown here.

Creating a node handle
----------------------

First of all, a ROS node is required for ROS communication.
For this reason, the necessary definitions must be included first. ::

 #include <ros/node_handle.h>

Then define a variable corresponding to the node handle. ::

 ros::NodeHandle node;

In the "configure" function, create the node handle by the following code. ::

 node.reset(new ros::NodeHandle(config->body()->name()));

This process itself is done in step 1, but here the name of the target robot is given to the node handle as a namespace.
The namespace is not necessarily required, but it is provided here to make it easier to distinguish topics.
This is because including the robot name in the namespace implies that the topic is related to the state of the robot.

Here, ::

 config->body()->name()

is used to obtain the target robot name.

For more information on the config object, please refer to :ref:`simulation-implement-controller-simple-controller-class-supplement` .

In this sample, the model name is "Tank", so all the topic names created below will be prefixed with "/Tank".

Creating a publisher
--------------------

In ROS, topics are designed to be output by the corresponding publisher, and the Publisher class for this is defined in roscpp.
In the sample controller, the following member variable corresponds to the publisher. ::

 ros::Publisher jointStatePublisher;

In order to publish, you need to prepare a message (data) for the corresponding topic.
To do this, you must first determine the message type.
In this sample, we will use the sensor_msgs::JointState type defined in a standard ROS package.

To check the content of this type, execute the following command.

.. code-block:: sh

 rosmsg show sensor_msgs/JointState

You should see something like the following.

.. code-block:: none

 std_msgs/Header header
   uint32 seq
   time stamp
   string frame_id
 string[] name
 float64[] position
 float64[] velocity
 float64[] effort

Here, the "header" part is the header common to each ROS message, and is the same as the one included in the Joy topic used in step 1. The following part is the main body of the JointState type, where each member of name, position, velocity, and effot is defined. Each of them corresponds to the name of the joints, joint displacements, joint velocities, and joint efforts (torques or forces). All of these are arrays, and the number of elements will be stored for the number of joints the robot has. This message type can be used to output the state of the robot's joints.

In order to use this message type from C++ code, define a variable of the class corresponding to the message type. If the message type is already installed as a package, the header file for using it in C++ should also be installed. In that case, there will be a C++ class that corresponds directly to the name of the message type.

In order to use the JointState type here, include the corresponding header first. ::

 #include <sensor_msgs/JointState.h>

You can see that the file path of the header corresponds directly to the type name of the message registered in ROS.

Then, the variable corresponding to this type is defined. ::

 sensor_msgs::JointState jointState;

This C++ type with the namespace also corresponds to the ROS message type name almost exactly.

.. note:: Although this sample uses an existing message type, it is also possible to use a message type that you have defined yourself. Please refer to the manual of roscpp for how to do so.

Now, let's create a publisher to output data of this message type.
The following code in the "configure" function will do this. ::

 jointStatePublisher = node->advertise<sensor_msgs::JointState>("joint_state", 1);

In this way, the publisher can be created using the advertise function of the node handle. This function is a template function that takes a message type as an argument, and by specifying the JointState type in this way, you can generate a publisher that outputs a JointState message.

The first argument of the function is the topic name. The actual topic name is concatenated with "Tank", which is set as the namespace of the node, to become "/Tank/joint_state".

The second argument specifies the size of the queue to be used for output. If you want to output a large number of messages in a short period of time, and you do not want to miss any messages, increase the size of the queue. If this is not necessary and the receiver of the messages only needs to get the latest message at each point in time, specifying a queue size of 1 may be appropriate. In this sample, we don't expect to miss anything, so we set the queue size to 1.

Now, we have created a publisher that outputs messages of the JointState type.

.. _ros_tank_tutorial_publish_joint_state:

Publishing a joint state
------------------------

The flow of the process to output the joint state is as follows:

1. Get the state of the robot's joints
2. Copy the state to a variable of the JointState type
3. Output a message of the JointState type using a publisher

All of these are done in the "control" function of the simple controller, which makes it possible to output the state of the joints periodically and repeatedly while the robot is running.

Note that preparation for the above process is also necessary.
It is implemented in the "initialize" function.

First, the following code is implemented. ::

 int n = ioBody->numJoints();
 jointState.name.resize(n);
 jointState.position.resize(n);
 jointState.velocity.resize(n);
 jointState.effort.resize(n);

In this code, the number of joints of the robot is obtained, and the size of the array of each member of JointState is allocated for the number of joints.
Since the number of joints of the robot does not change during the control, this process is done only once during initialization. Such a process is implemented in the initialize function. The number of joints of the Tank robot used in this sample is 2 axes: turret yaw axis and barrel pitch axis.

Next, configure the settings for inputting the robot's state to the simple controller.
This is handled by the following code, which is also implemented in the initialize function. ::

 for(int i=0; i < n; ++i) {
     auto joint = ioBody->joint(i);
     io->enableInput(joint, JointDisplacement | JointVelocity | JointEffort);
     jointState.name[i] = joint->name();
  }

This specifies the state values to be input from the robot to the simple controller using the enableInput function of SimpleController's ref:`simulation-implement-controller-simple-controller-io` . By specifying JointDisplacement, JointVelocity, and JointEffort, the joint displacements, joint velocities, and joint torques are input for the two axes of the turret and barrel. In addition, the joint name is obtained and copied to the name member of the JointState message. This allows the receiver of the topic to obtain the joint name as well.

For details on input/output settings, see :ref:`simulation-implement-controller-io-by-body-object` .

At the end of the initialization, the time-related variables are initialized as follows. ::

 time = 0.0;
 timeStep = io->timeStep();
 double frequency = 50.0;
 cycleTime = 1.0 / frequency;
 timeCounter = 0.0;

The values set here will be referenced in the control function.

The value set for frequency corresponds to the frame rate of publishing, which determines how often the publication made.
Increasing this value will result in a state output with higher temporal resolution, but it will also increase the communication cost, so it is necessary to adjust it appropriately based on the network environment and the overall communication volume of the system.

This completes the preparation. The next step is to implement the above steps 1 to 3 in the control function.

First of all, the control function is implemented in the following structure to adjust the cycle of the state output. ::

 time += timeStep;
 timeCounter += timeStep;
 
 if(timeCounter >= cycleTime) {
            
     // Create and publish the JointState message
     ...     

     timeCounter -= cycleTime;
 }

Here, the time variable contains the time in seconds counted from the start of the simulation.
In order to adjust the cycle of the state output, timeCounter contains the elapsed time since the last output. ::

 if(timeCounter >= cycleTime) {

is a condition to output the state only when the timeCounter reaches the cycleTime corresponding to the cycle.
In general, the control function is executed in the control cycle of the robot, but that is often too short a cycle for state output. Therefore, in this sample, a separate cycle for the state output is set so that the state is output at an appropriate cycle. This kind of adjustment of the output cycle should be done for each topic based on its type and usage, so please keep this in mind.

When timeCounter reaches the set cycle and outputs the state, reset the timeCounter by ::

 timeCounter -= cycleTime;

With the adjusted in this way, the actual state output is done by the code in this if block.

First of all ::

 jointState.header.stamp.fromSec(time);

sets the current time to the stamp in the header of the JointState message.

Then, the joint angles, joint angular velocities, and joint torques of the turret and barrel axes are copied to the corresponding members of the JointState type by the following code.  ::

 for(int i=0; i < ioBody->numJoints(); ++i) {
     auto joint = ioBody->joint(i);
     jointState.position[i] = joint->q();
     jointState.velocity[i] = joint->dq();
     jointState.effort[i] = joint->u();
 }

The current joint state is now stored in the jointState variable. All that remains is to publish it. This is done by giving a message to the publish function of the publisher object and calling ::
            
 jointStatePublisher.publish(jointState);

and you are good to go.

This will cause the topic "/Tank/joint_state" to be published with a message of the JointState type at each specified period.

Introducing a controller for state output
-----------------------------------------

Let's build the controller corresponding to the above source code and introduce it into the simulation project.
The procedure is the same as the RttTankController introduced in Step 1.

First, create the above source code in the src directory with the file name "RttJointStatePublisher.cpp". Then, add the following description to CMakeLists.txt in the same src directory to build this controller.

.. code-block:: cmake

 choreonoid_add_simple_controller(RttJointStatePublisher RttJointStatePublisher.cpp)
 target_link_libraries(RttJointStatePublisher ${roscpp_LIBRARIES})

Doing this and catkin build again should build RttJointStatePublisher and make it available if there are no errors in the source code or CMakeLists.txt. If there are any errors in the build, please correct them accordingly.

If the build is successful, add RttJointStatePublisher to the simulation project as :ref:`ros_tank_tutorial_step1_introduce_controller` in step 1.

Since the simple controller can be used in multiple combinations, you can configure the item tree as follows.

.. code-block:: none

 + World
   + Tank
     - RttTankController
     - RttJointStatePublisher
   - Labo1
   - AISTSimulator

To add the RttJointStatePublisher, create a SimpleController item as a child item of the Tank item and select "RttJointStatePublisher.so" in its "Controller Module" property dialog.

Let's save the project in this state. In this tutorial, we will save a separate project file for each step. So let's save this project as "step2.cnoid". Also, let's create a launch file for step 2. For now, create "step2.launch" by copying "step1.launch", which is the launch file created for step 1. Then, modify the part of "step1.cnoid" to "step2.cnoid". Then, "step2.launch" will look like the following.

.. code-block:: xml

 <launch>
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
         args="$(find choreonoid_ros_tank_tutorial)/project/step2.cnoid --start-simulation" />
   <node pkg="rqt_graph" name="rqt_graph" type="rqt_graph" />
 </launch>

After this process, the package for this tutorial will have the following file structure.

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   + launch
     - step1.launch
     - step2.launch
   + project
     - step1.cnoid
     - step2.cnoid
   + src
     - CMakeLists.txt
     - RttTankController.cpp
     - RttJointStatePublisher.cpp


Check the output topic
----------------------

.. highlight:: sh

Let's run the simulation project and check if the topics of the joint states are output.

First, launch the project in step 2 with the following command. ::

 roslaunch choreonoid_ros_tank_tutorial step2.launch

Then you will be able to control the Tank robot with the gamepad as in step 1.

Now, let's prepare a terminal for command input and check the topic from the terminal.
First, run the following command to display the list of available topics. ::

 rostopic list

You should see something like the following.

.. code-block:: none

 /Tank/joint_state
 /joy
 /rosout
 /rosout_agg
 /statistics

Here, "/Tank/joint_state" corresponds to the topic we implemented.
If you don't see this topic, there is a mistake somewhere in the source code or in your project, so please check it.

Next, let's check the information on this topic with the following command. ::

 rostopic info /Tank/joint_state

You should see something like the following.

.. code-block:: none

 Type: sensor_msgs/JointState
 
 Publishers: 
  * /choreonoid (http://host:38755/)
 
 Subscribers: None

This will tell you the following:

* The message type is sensor_msgs/JointState.
* The node that is the publisher of this topic is the node "/choreonoid" in the displayed host.
* There is no subscriber for this topic.

As for the subscriber, we have not made any connections yet, so we get this result.

Let's also check the content of the published message. Enter the following command: ::

 rostopic echo /Tank/joint_state

You will continue to see the following text output.

.. code-block:: none

 header: 
   seq: 31
   stamp: 
     secs: 1
     nsecs: 600000000
   frame_id: ''
 name: 
   - TURRET_Y
   - TURRET_P
 position: [1.6122377450560194e-09, -0.00979137291587475]
 velocity: [-2.827205716540265e-10, -6.034345222794471e-05]
 effort: [-3.091940828686953e-07, 1.9612950742218773]
 ---

While continuing this output, try moving the gun barrel with the gamepad. You will see the values of position, velocity, and effort change. The units are [rad], [rad/sec], and [N・m], respectively.

By the way, if you execute ::

 rostopic info /Tank/joint_state

again on another terminal without stopping rostopic echo, you will find that "Subscribers:" is no longer None. This subscriber corresponds to the "rostopic echo".

We can now confirm that the joint state is successfully output as a ROS topic.

Displaying joint angle graph
----------------------------

Now that the state of the joint can be output as a ROS topic, you can use this information in conjunction with various ROS nodes/tools. As a very simple example, let's use the rqt_plot tool to display a graph of joint angles.

With the previous simulation running, enter the following command from a terminal. ::

 rosrun rqt_plot rqt_plot /Tank/joint_state/position[0] /Tank/joint_state/position[1]

You will see a window like the one below.

.. image:: images/rqt_plot1.png

Now, check the "autoscroll" checkbox in the upper right corner, and move the gun barrel with the gamepad.
You will see changes in the joint angles drawn as a graph.

.. image:: images/rqt_plot2.png

In the above figure, the blue line corresponds to the yaw axis and the red axis to the pitch axis.

Finally, let's include the display of rqt_plot in the launch file by adding the corresponding item to step2.launch as follows:

.. code-block:: xml

 <launch>
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
         args="$(find choreonoid_ros_tank_tutorial)/project/step2.cnoid --start-simulation" />
   <node pkg="rqt_graph" name="rqt_graph" type="rqt_graph" />
   <node pkg="rqt_plot" name="rqt_plot" type="rqt_plot"
         args="/Tank/joint_state/position[0] /Tank/joint_state/position[1]" />
 </launch>

Now, if you start the launch file, the graph will be displayed by rqt_graph.

This concludes step 2.
