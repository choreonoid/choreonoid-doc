Step 4: Output the state of the acceleration sensor and rate gyro
=================================================================

In step 4, we will create a program that outputs the state of the acceleration sensor and rate gyro in the Tank robot as a ROS topic.

.. contents::
   :local:

.. highlight:: c++

Overview
--------

In addition to the joint states such as joint angles of a robot, which is one of the basic state values that the robot has, the robot has various other state values that can be obtained from various sensors. As an example, in Step 4, we will learn how to output the state values of the acceleration sensor and rate gyro of the Tank robot.

These sensors are usually modeled as the object type called "device" inside Choreonoid, and by publishing the state of the corresponding device object, it is possible to output the sensed state values.
ROS defines the standard message type sensor_msgs/Imu as an appropriate type for outputting these sensor values, so this step will use it to publish the state this time.

Once you have learned this method, you will be able to output the status of other sensors and devices in the same way.

Controller for outputting sensing values
----------------------------------------

The process in this step will also be realized by creating a simple controller for this purpose.
The source code is shown below.

.. code-block:: c++
 :linenos:

 #include <cnoid/SimpleController>
 #include <cnoid/AccelerationSensor>
 #include <cnoid/RateGyroSensor>
 #include <ros/node_handle.h>
 #include <sensor_msgs/Imu.h>

 using namespace std;
 using namespace cnoid;

 class RttImuStatePublisher : public SimpleController
 {
     std::unique_ptr<ros::NodeHandle> node;
     ros::Publisher imuPublisher;
     sensor_msgs::Imu imu;
     AccelerationSensorPtr accelSensor;
     RateGyroSensorPtr gyro;
     Vector3 dv_sum;
     Vector3 w_sum;
     int sensingSteps;
     double time;
     double timeStep;
     double cycleTime;
     double timeCounter;

 public:
     virtual bool configure(SimpleControllerConfig* config) override
     {
	 node.reset(new ros::NodeHandle(config->body()->name()));
	 imuPublisher = node->advertise<sensor_msgs::Imu>("imu", 1);
	 return true;
     }

     virtual bool initialize(SimpleControllerIO* io) override
     {
	 accelSensor = io->body()->findDevice<AccelerationSensor>();
	 gyro = io->body()->findDevice<RateGyroSensor>();

	 io->enableInput(accelSensor);
	 io->enableInput(gyro);

	 dv_sum.setZero();
	 w_sum.setZero();
	 sensingSteps = 0;

	 for(int i=0; i < 9; ++i){
	     imu.orientation_covariance[i] = 0.0;
	     imu.angular_velocity_covariance[i] = 0.0;
	     imu.linear_acceleration_covariance[i] = 0.0;
	 }
	 imu.orientation_covariance[0] = -1.0;
	 imu.orientation.x = 0.0;
	 imu.orientation.y = 0.0;
	 imu.orientation.z = 0.0;
	 imu.orientation.w = 0.0;

	 time = 0.0;
	 timeStep = io->timeStep();
	 const double frequency = 20.0;
	 cycleTime = 1.0 / frequency;
	 timeCounter = 0.0;

	 return true;
     }

     virtual bool control() override
     {
	 dv_sum += accelSensor->dv();
	 w_sum += gyro->w();
	 ++sensingSteps;

	 time += timeStep;
	 timeCounter += timeStep;

	 if(timeCounter >= cycleTime){
	     imu.header.stamp.fromSec(time);

	     auto dv = dv_sum / sensingSteps;
	     imu.linear_acceleration.x = dv.x();
	     imu.linear_acceleration.y = dv.y();
	     imu.linear_acceleration.z = dv.z();
	     dv_sum.setZero();

	     auto w = w_sum / sensingSteps;
	     imu.angular_velocity.x = w.x();
	     imu.angular_velocity.y = w.y();
	     imu.angular_velocity.z = w.z();
	     w_sum.setZero();

	     sensingSteps = 0;

	     imuPublisher.publish(imu);

	     timeCounter -= cycleTime;
	 }

	 return true;
     }
 };

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttImuStatePublisher)

The following sections describe the key points of the implementation of this controller, and the contents of the code is explained accordingly.

sensor_msgs/Imu type message
----------------------------

In this step, we will use a message of the `sensor_msgs/Imu <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html>`_ type, which is one of the ROS standard message types.
This message type is used to store the state values of the Inertial Measurement Unit (IMU).
The IMU is basically a three-axis gyroscope and accelerometer that determine three-dimensional angular velocity and acceleration, and is also used for further estimation of posture and position.

The contents of the Imu type can also be checked using the following command. ::

 rosmsg show sensor_msgs/Imu

As shown in this command, this type is defined as follows: ::

 std_msgs/Header header
   uint32 seq
   time stamp
   string frame_id
 geometry_msgs/Quaternion orientation
   float64 x
   float64 y
   float64 z
   float64 w
 float64[9] orientation_covariance
 geometry_msgs/Vector3 angular_velocity
   float64 x
   float64 y
   float64 z
 float64[9] angular_velocity_covariance
 geometry_msgs/Vector3 linear_acceleration
   float64 x
   float64 y
   float64 z
 float64[9] linear_acceleration_covariance

In this step, we will use the "angular_velocity" field for the gyro and the "linear_acceleration" field for the acceleration sensor. The state values of the corresponding sensors are stored into the x, y, and z components of those fields. 
In the "angular_velocity_covariance" and "linear_acceleration_covariance" fields, the covariance matrix corresponding to each element is stored, but we will assume that these values are unknown and fill the elements with 0.

In the "orientation" field, the estimated values of the orientation is store, but we assume that this section is not covered by the IMU on the Tank robot.
According to the reference manual, this can be expressed by setting the first element of orientation_covariance to -1.

In the source of the above controller, first the definition of the Imu type is included so that it can be used in the code. ::

 #include <sensor_msgs/Imu.h>

Then, to publish messages of this type, the following variable is defined as a member variable of the controller. ::

 sensor_msgs::Imu imu;

Then, in the "configure" function, a publisher to output the topic "imu" of this message type is created. ::

 imuPublisher = node->advertise<sensor_msgs::Imu>("imu", 1);

In the following code of the "initialize" function, we set the values to invalidate the part of the Imu type elements that are not used this time. ::

 for(int i=0; i < 9; ++i){
     imu.orientation_covariance[i] = 0.0;
     imu.angular_velocity_covariance[i] = 0.0;
     imu.linear_acceleration_covariance[i] = 0.0;
 }
 imu.orientation_covariance[0] = -1.0;
 imu.orientation.x = 0.0;
 imu.orientation.y = 0.0;
 imu.orientation.z = 0.0;
 imu.orientation.w = 0.0;

The covariance matrix is set to unknown by filling each covariance element with 0, and the entire value of orientation is also set to unknown by setting -1 to the first element of covariance for orientation.

Getting the sensor device
-------------------------

The angular velocity and linear acceleration values to be stored in the Imu message are obtained from the rate gyro and acceleration sensor devices provided by the Tank robot.

First of all, these devices are defined as child elements of CHASSIS links in "TankBody.body", which is a part of the model file of the Tank robot, as follows.

.. code-block:: yaml

 links:
   -
     name: CHASSIS
     ...
 
     elements:
       ...
 
       -
         type: AccelerationSensor
         name: ACCEL_SENSOR
         id: 0
       -
         type: RateGyroSensor
         name: GYRO
         id: 0
    ...

By this description, an acceleration sensor and a rate gyro sensor are mounted one by one as devices at the origin (center) of the Tank robot body. The states of these devices are acquired in the simple controller using the method described in :ref:`simulation-device` .

First, the corresponding device objects are obtained by the method described in :ref:`simulation-obtain-device-object` .

For this purpose, the headers of the device types to be used are included. ::

 #include <cnoid/AccelerationSensor>.
 #include <cnoid/RateGyroSensor>.

These headers make available the definitions of the AccelerationSensor type and the RateGyroSensor type, respectively.

In the definition of the controller class, smart pointer variables to hold the objects of these sensor types are defined as a member variables. ::

 AccelerationSensorPtr accelSensor;
 RateGyroSensorPtr gyro;

Then, in the "initialize" function of the controller, these variables are initialized with the corresponding device objects for input and output (I/O). ::

 accelSensor = io->body()->findDevice<AccelerationSensor>();
 gyro = io->body()->findDevice<RateGyroSensor>();

As in the previous controller, the body object for I/O is obtained from the :ref:`simulation-implement-controller-simple-controller-io` . By specifying the device type to be acquired in the findDevice function, the corresponding sensor device is acquired.

In this part, if there are multiple devices of the same type, the device name must be specified as follows. ::

 accelSensor = io->body()->findDevice<AccelerationSensor>("ACCEL_SENSOR");

In this way, the device object must be identified by its attribute such as name.
However, this is not necessary for the Tank robot.

For these devices, ::

 io->enableInput(accelSensor);
 io->enableInput(gyro);

enable the input.
This enables the device I/O objects to reflect the state of the corresponding sensors as they are updated during the simulation.

Obtaining sensor state values
-----------------------------

The state values of the sensors are obtained in the control function of the controller.
First, the value of the acceleration sensor is obtained by the following code. ::

 dv_sum += accelSensor->dv();

Here, the value of member variable dv_sum is updated.
dv_sum is a Vector3 type variable that stores the accumulated acceleration values.
The acceleration value for each control loop is accumulated here until the next publish.

The acceleration value may change significantly at each time step of the simulation, and if it is output as is, the change may be noisier than that of a real sensor. Also, the frequency of publishing topics is generally longer than the simulation time step.
Considering this, one way to stabilize the output acceleration value is to average the values from one publish to the next. The variable dv_sum is used for this purpose.

For the rate gyro, the angular velocity values are obtained and accumulated in a similar manner.
This will result in the following code. ::

 w_sum += gyro->w();

Then, the number of times of accumulations is recorded in the sensingSteps member variable of integer type. ::

 ++sensingSteps;

The actual averaged values to be output can be calculated by dividing each accumulated value by sensingSteps.
This calculation is represented by the following code. ::

 auto dv = dv_sum / sensingSteps;
 auto w = w_sum / sensingSteps;

Publishing these values will be explained later.

Note that The values of these variables are all initialized to zero in the following code in the initialize function. ::

 dv_sum.setZero();
 w_sum.setZero();
 sensingSteps = 0;

.. note:: The averaging method applied here is designed to stabilize the output value with minimum implementation, and is not necessarily the best method. Real IMU sensors may include various correction processes to improve accuracy, and it may be better to introduce the same correction processes as those used in real sensors, if necessary. It is possible to introduce such processing into the simulation process inside Choreonoid as part of the simulation of the sensor devices, but since such processing is not included in the current version, it is necessary to process the values appropriately on the side where the values are used as in this example.

Publishing sensing values
-------------------------

In publishing the sensing values, as in :ref:`ros_tank_tutorial_publish_joint_state` in Step 2, it is necessary to properly handle the time-related aspects such as the timestamp to be given to the message and the timing (cycle) of the publication.
This part is handled in the same way as in Step 2, but we will explain this part of the code again in this step.

First, the following four member variables are defined to store the time-related information for publishing. ::

 double time;
 double timeStep;
 double cycleTime;
 double timeCounter;

time is the elapsed time after the controller starts running, timeStep is the time step of the control loop, cycleTime is the cycle in which publishing is performed, and timeCounter is the time counter that determines if the next publishing cycle has been reached.

These variables are first initialized in the initialize function as follows. ::

  time = 0.0;
  timeStep = io->timeStep();
  const double frequency = 20.0;
  cycleTime = 1.0 / frequency;
  timeCounter = 0.0;

Here, we set the cycleTime to publish 20 times per second.
This value should be adjusted appropriately in consideration of the communication environment and the use of the topic.

Publishing is done in the "control" function.
Note that, instead of publishing every time the control function is executed, the timing should be adjusted so that it publishes in the cycle of the cycleTime specified above.
The code for this adjustment is as follows. ::

  time += timeStep;
  timeCounter += timeStep;

  if(timeCounter >= cycleTime){

      // Publish here

      timeCounter -= cycleTime;
  }

By doing this, the cycle of publishing will be adjusted to match the cycleTime.

In this if block, the contents of the Imu type variable are updated and the publishment is performed.
First, the time information contained in the header is updated. ::

  imu.header.stamp.fromSec(time);

Then, for acceleration, we calculate the averaged value using the method described above. ::

  auto dv = dv_sum / sensingSteps;

Assign the value of dv to each element of linear_acceleration of the Imu type variable. ::

  imu.linear_acceleration.x = dv.x();
  imu.linear_acceleration.y = dv.y();
  imu.linear_acceleration.z = dv.z();

Clear the value of dv_sum for the next publishment. ::

  dv_sum.setZero();

Perform the same process for angular velocity as for acceleration. ::

  auto w = w_sum / sensingSteps;
  imu.angular_velocity.x = w.x();
  imu.angular_velocity.y = w.y();
  imu.angular_velocity.z = w.z();
  w_sum.setZero();

For the next publishment, the value of sensingSteps is cleared. ::

  sensingSteps = 0;

Now the contents of the Imu message variable have been updated to the latest state.
Finally, the message is actually published by the following code. ::

  imuPublisher.publish(imu);

Introducing the controller
--------------------------

As in the previous steps, build the controller corresponding to the above source code and introduce it into the simulation project.

First, create the above source code in the src directory with the file name "RttImuStatePublisher.cpp".
Then, add the following description to CMakeLists.txt in the same directory.

.. code-block:: cmake

 choreonoid_add_simple_controller(RttImuStatePublisher RttImuStatePublisher.cpp)
 target_link_libraries(RttImuStatePublisher ${roscpp_LIBRARIES})

Then executing the "catkin build" command will build RttImuStatePublisher. If the build is successful, add the RttImuStatePublisher to the project as in the previous steps. Specifically, add this controller to the project created in step 2, and configure the item tree as follows.

.. code-block:: none

 + World
   + Tank
     - RttTankController
     - RttJointStatePublisher
     - RttImuStatePublisher <- Add this
   - Labo1
   - AISTSimulator

The RttImuStatePublisher to be added here is a SimpleController type item, and specify "RttJointStatePublisher.so" as its "controller module". Save this project with the file name "step4.cnoid".

Also, create a launch file to run this project with the following contents as "step4.launch".

.. code-block:: xml

 <launch>
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
	 args="$(find choreonoid_ros_tank_tutorial)/project/step4.cnoid --start-simulation" />
   <node pkg="rqt_graph" name="rqt_graph" type="rqt_graph" />
 </launch>

After this step, the package for this tutorial will have the following file structure.

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   + launch
     - step1.launch
     - step2.launch
     - step3.launch
     - step4.launch
   + project
     - step1.cnoid
     - step2.cnoid
     - step3.cnoid
     - step4.cnoid
   + src
     - CMakeLists.txt
     - RttTankController.cpp
     - RttJointStatePublisher.cpp
     - RttJointStateSubscriber.cpp
     - RttImuStatePublisher.cpp

.. _ros_tank_tutorial_step3_check_topic_values:

Checking the output topics
--------------------------

.. highlight:: sh

When you run step4.launch, the simulation will start and the IMU topics will be added by RttImuStatePublisher.
To confirm this, execute ::

 rostopic list 

and the following item should be displayed.

.. code-block:: none

 /Tank/imu

This is the IMU topic. Then execute ::

 rostopic info /Tank/imu

and you will see

.. code-block:: none

 Type: sensor_msgs/Imu
 
 Publishers: 
  * /choreonoid (http://rynoid:44641/)
 
 Subscribers: None

This confirms that the message type is actually "sensor_msgs/Imu".
If you execute ::

 rostopic echo /Tank/imu

, it will continue to display the sensor status values as follows.

.. code-block:: none

 header: 
   seq: 3399
   stamp: 
     secs: 170
     nsecs:         0
   frame_id: ''
 orientation: 
   x: 0.0
   y: 0.0
   z: 0.0
   w: 0.0
 orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 angular_velocity: 
   x: -1.3141583564318781e-09
   y: -6.139951539231158e-12
   z: -1.0749827270382294e-13
 angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 linear_acceleration: 
   x: -1.220294155439848e-08
   y: 4.219067333397275e-09
   z: 9.806650065226014
 linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 ---

Try to move the chassis of the Tank robot in this state, and you will see the values of angular_velocity and linear_acceleration change. The units are [rad/sec] and [m/s^2], respectively.

The turning velocity of the tank corresponds to the z-element of angular_velocity, and the acceleration of the tank in the forward/backward direction corresponds to the x-element of linear_acceleration, so it is relatively easy to understand the changes in these values caused by turning the tank or moving it forward/backward.

Displaying acceleration / angular velocity graph
--------------------------------------------------

Similar to the joint angles in Step 2, let's display the acceleration and angular velocity in a graph.
With the simulation of this step running, enter the following command from a terminal. ::

 rosrun rqt_plot rqt_plot /Tank/imu/linear_acceleration

This will plot the X, Y, and Z axis elements of acceleration on a graph.
It is easy to see the change in the X-axis element when the tank chassis is moved back and forth.
You can also see a large change in acceleration on the graph when the tank collides with the environment.

In the same way, angular velocity can be plotted using the following command. ::

 rosrun rqt_plot rqt_plot /Tank/imu/angular_velocity

This will plot the X, Y, and Z axis elements of the angular velocity on a graph.
It is easy to see the change in the Z-axis element caused by turning the Tank chassis.
