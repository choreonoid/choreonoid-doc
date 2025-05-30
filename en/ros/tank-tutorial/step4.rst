Step 4: Outputting Acceleration Sensor and Rate Gyro States
============================================================

In Step 4, we'll create a program that outputs the Tank robot's acceleration sensor and rate gyro states as ROS topics.

.. contents::
   :local:

.. highlight:: c++

Overview
--------

Joint states such as joint angles are fundamental robot state values, but robots also have various other state values obtainable from different sensors. In Step 4, we'll learn how to output state values from the Tank robot's acceleration sensor and rate gyro as examples.

These sensors are typically modeled as "device" objects within Choreonoid. By publishing the corresponding device object's state, you can output the sensed values. ROS provides the standard message type sensor_msgs/Imu as appropriate for outputting these sensor values, so we'll use it to publish the states in this step.

Once you master this method, you'll be able to output other sensors' and devices' states using the same approach.

Sensor Value Output Controller
------------------------------

We'll implement this step's processing by creating a simple controller.
Here's the source code:

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

The following sections explain the key implementation points of this controller and describe the code content accordingly.

sensor_msgs/Imu Type Messages
-----------------------------

This step uses the `sensor_msgs/Imu <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html>`_ message type, one of ROS's standard message types. This type stores state values from an Inertial Measurement Unit (IMU). An IMU fundamentally consists of three-axis gyroscopes and accelerometers that measure three-dimensional angular velocity and acceleration, and is also used for estimating attitude and position.

You can check the Imu type's contents with this command: ::

 rosmsg show sensor_msgs/Imu

This command displays the following type definition: ::

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

We'll use the "angular_velocity" field for the gyro and the "linear_acceleration" field for the acceleration sensor, storing the corresponding sensor state values in their x, y, and z components. The "angular_velocity_covariance" and "linear_acceleration_covariance" fields store covariance matrices for each element, but we'll assume these values are unknown and fill the elements with 0.

The "orientation" field stores orientation estimates, but we'll assume the Tank robot's IMU doesn't provide this. According to the reference manual, setting orientation_covariance's first element to -1 indicates this.

In the controller source above, we first include the Imu type definition to use it in the code: ::

 #include <sensor_msgs/Imu.h>

To publish messages of this type, we define the following member variable in the controller: ::

 sensor_msgs::Imu imu;

In the configure function, we create a publisher for the "imu" topic of this message type: ::

 imuPublisher = node->advertise<sensor_msgs::Imu>("imu", 1);

In the initialize function, the following code sets values to invalidate unused Imu type elements: ::

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

We set the covariance matrices to unknown by filling each covariance element with 0, and set the entire orientation value to unknown by setting orientation_covariance's first element to -1.

Obtaining Sensor Devices
------------------------

The angular velocity and acceleration values stored in the Imu message come from the Tank robot's rate gyro and acceleration sensor devices.

These devices are defined as child elements of the CHASSIS link in "TankBody.body", part of the Tank robot's model file:

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

This defines one acceleration sensor and one rate gyro as devices at the Tank robot body's origin (center). We obtain these devices' states in the simple controller using the method described in :ref:`simulation-device`.

First, we obtain the corresponding device objects using the method in :ref:`simulation-obtain-device-object`.

We include the necessary device type headers: ::

 #include <cnoid/AccelerationSensor>
 #include <cnoid/RateGyroSensor>

These headers provide the AccelerationSensor and RateGyroSensor type definitions.

In the controller class definition, we define smart pointer member variables to hold these sensor type objects: ::

 AccelerationSensorPtr accelSensor;
 RateGyroSensorPtr gyro;

In the controller's initialize function, we obtain the I/O device objects: ::

 accelSensor = io->body()->findDevice<AccelerationSensor>();
 gyro = io->body()->findDevice<RateGyroSensor>();

As in previous controllers, we obtain the I/O Body object from :ref:`simulation-implement-controller-simple-controller-io`. By specifying the device type in the findDevice function, we acquire the corresponding sensor device.

If multiple devices of the same type exist, you must specify the device name: ::

 accelSensor = io->body()->findDevice<AccelerationSensor>("ACCEL_SENSOR");

This identifies the device object by attributes such as name. However, this isn't necessary for the Tank robot since each device type appears only once.

For these devices: ::

 io->enableInput(accelSensor);
 io->enableInput(gyro);

enables input. This allows the I/O device objects to reflect the corresponding sensors' states as they update during simulation.
This enables the device I/O objects to reflect the state of the corresponding sensors as they are updated during the simulation.

Obtaining Sensor State Values
-----------------------------

Sensor state values are obtained in the controller's control function.
First, we obtain the acceleration sensor value with: ::

 dv_sum += accelSensor->dv();

This updates the member variable dv_sum. dv_sum is a Vector3 variable that accumulates acceleration values. Each control loop's acceleration value accumulates here until the next publish.

Acceleration values can change significantly at each simulation timestep. Outputting them directly could result in noisier changes than real sensors produce. Additionally, topic publishing frequency is generally lower than the simulation timestep frequency. Considering this, one stabilization method averages values between publications. The dv_sum variable serves this purpose.

For the rate gyro, we obtain and accumulate angular velocity values similarly: ::

 w_sum += gyro->w();

We then record the accumulation count in the integer member variable sensingSteps: ::

 ++sensingSteps;

The averaged output values are calculated by dividing accumulated values by sensingSteps: ::

 auto dv = dv_sum / sensingSteps;
 auto w = w_sum / sensingSteps;

We'll explain publishing these values later.

Note that these variables are initialized to zero in the initialize function: ::

 dv_sum.setZero();
 w_sum.setZero();
 sensingSteps = 0;

.. note:: The averaging method applied here provides basic output stabilization with minimal implementation but isn't necessarily optimal. Real IMU sensors may include various correction processes for accuracy improvement. If necessary, implementing similar correction processes might be beneficial. While such processing could be integrated into Choreonoid's sensor device simulation, the current version doesn't include it, so users must process values appropriately as shown in this example.

Publishing Sensor Values
------------------------

When publishing sensor values, as with :ref:`ros_tank_tutorial_publish_joint_state` in Step 2, we must properly handle time-related aspects such as message timestamps and publication timing (cycles). While this is handled similarly to Step 2, we'll explain this code portion again.

First, we define four member variables for time-related publishing information: ::

 double time;
 double timeStep;
 double cycleTime;
 double timeCounter;

Here, time is the elapsed time since controller startup, timeStep is the control loop's timestep, cycleTime is the publishing cycle, and timeCounter determines whether the next publishing cycle has been reached.

These variables are initialized in the initialize function: ::

  time = 0.0;
  timeStep = io->timeStep();
  const double frequency = 20.0;
  cycleTime = 1.0 / frequency;
  timeCounter = 0.0;

Here we set cycleTime to publish 20 times per second. Adjust this value appropriately considering the communication environment and topic usage.

Publishing occurs in the control function. Rather than publishing every control function execution, we adjust timing to match the specified cycleTime. The timing adjustment code is: ::

  time += timeStep;
  timeCounter += timeStep;

  if(timeCounter >= cycleTime){

      // Publish here

      timeCounter -= cycleTime;
  }

This adjusts the publishing cycle to match cycleTime.

Within this if block, we update the Imu type variable's contents and perform publishing. First, we update the header's time information: ::

  imu.header.stamp.fromSec(time);

For acceleration, we calculate the averaged value using the previously described method: ::

  auto dv = dv_sum / sensingSteps;

We assign dv's values to the Imu variable's linear_acceleration elements: ::

  imu.linear_acceleration.x = dv.x();
  imu.linear_acceleration.y = dv.y();
  imu.linear_acceleration.z = dv.z();

We clear dv_sum for the next publication: ::

  dv_sum.setZero();

We perform the same process for angular velocity: ::

  auto w = w_sum / sensingSteps;
  imu.angular_velocity.x = w.x();
  imu.angular_velocity.y = w.y();
  imu.angular_velocity.z = w.z();
  w_sum.setZero();

We also clear sensingSteps for the next publication: ::

  sensingSteps = 0;

The Imu message variable now contains the latest state. Finally, we publish the message: ::

  imuPublisher.publish(imu);

Introducing the Controller
--------------------------

As in previous steps, build the controller from the above source code and introduce it to the simulation project.

First, create the source code in the src directory as "RttImuStatePublisher.cpp". Then add the following to CMakeLists.txt in the same directory:

.. code-block:: cmake

 choreonoid_add_simple_controller(RttImuStatePublisher RttImuStatePublisher.cpp)
 target_link_libraries(RttImuStatePublisher ${roscpp_LIBRARIES})

Execute catkin build to build RttImuStatePublisher. After successful building, add RttImuStatePublisher to the project as in previous steps. Specifically, add this controller to the Step 2 project, configuring the item tree as:

.. code-block:: none

 + World
   + Tank
     - RttTankController
     - RttJointStatePublisher
     - RttImuStatePublisher <- Add this
   - Labo1
   - AISTSimulator

The RttImuStatePublisher added here is a SimpleController item. Specify "RttImuStatePublisher.so" as its "Controller Module". Save this project as "step4.cnoid".

Also create a launch file "step4.launch" with the following content:

.. code-block:: xml

 <launch>
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
	 args="$(find choreonoid_ros_tank_tutorial)/project/step4.cnoid --start-simulation" />
   <node pkg="rqt_graph" name="rqt_graph" type="rqt_graph" />
 </launch>

After completing this work, the tutorial package has the following file structure:

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   - CMakeLists.txt
   - package.xml
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

Verifying Output Topics
-----------------------

.. highlight:: sh

Running step4.launch starts the simulation with RttImuStatePublisher adding IMU topics. To verify this, execute: ::

 rostopic list 

You should see:

.. code-block:: none

 /Tank/imu

This is the IMU topic. Next, execute: ::

 rostopic info /Tank/imu

This displays:

.. code-block:: none

 Type: sensor_msgs/Imu
 
 Publishers: 
  * /choreonoid (http://rynoid:44641/)
 
 Subscribers: None

This confirms the message type is actually "sensor_msgs/Imu". Execute: ::

 rostopic echo /Tank/imu

to continuously display sensor state values:

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

Move the Tank robot's chassis in this state to see angular_velocity and linear_acceleration values change. Units are [rad/sec] and [m/s²], respectively.

The tank's turning velocity corresponds to angular_velocity's z-component, and forward/backward acceleration corresponds to linear_acceleration's x-component, making these value changes relatively easy to understand when turning the tank or moving it forward/backward.

Displaying Acceleration and Angular Velocity Graphs
---------------------------------------------------

Similar to Step 2's joint angles, let's graph the acceleration and angular velocity. With this step's simulation running, enter from a terminal: ::

 rosrun rqt_plot rqt_plot /Tank/imu/linear_acceleration

This plots acceleration's X, Y, and Z axis components on a graph. The X-axis component changes are easily visible when moving the tank chassis forward and backward. You'll also see large acceleration changes when the tank collides with the environment.

Similarly, plot angular velocity with: ::

 rosrun rqt_plot rqt_plot /Tank/imu/angular_velocity

This plots angular velocity's X, Y, and Z axis components. The Z-axis component changes are easily visible when turning the Tank chassis.
