Controller Implementation
========================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

Controller Implementation
-------------------------

This section explains the basics of controller implementation.

The controller basically performs the following three operations, executing them repeatedly as a "control loop":

1. Input the robot's state
2. Perform control calculations
3. Output commands to the robot

These processes may be performed by a single controller or by combining multiple software components. Additionally, the process summarized as "control calculations" actually involves various processes such as recognition and motion planning, and may include input/output targeting things other than the robot. However, when viewed from the robot's perspective, what the controller ultimately does can be organized into the three processes above.

Thinking about it this way, a controller can be considered a software module equipped with interfaces for performing these three operations. While the actual API for this differs depending on the controller format, the essential parts are the same.

In the following, we will explain using the "SR1MinimumController" sample that was also used in :doc:`howto-use-controller`. The controller format is the "Simple Controller" format designed for Choreonoid samples, and the control content is simply maintaining the robot's posture through PD control of the joints. The description language is C++.

When actually developing a controller, you should replace the basic concepts described through this sample with your desired controller format and control content. Generally, developing robot controllers requires various knowledge and skills related to control, programming, hardware, etc. Since many of these are outside the scope of this manual, please work on them separately according to what you want to do.


Sample Controller Source Code
-----------------------------

First, here is the source code for SR1MinimumController. This source code is in the file "SR1MinimumController.cpp" in the "sample/SimpleController" directory of the Choreonoid source. ::

 #include <cnoid/SimpleController>
 #include <vector>
 
 using namespace cnoid;
 
 const double pgain[] = {
     8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
     3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
     8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
     3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
     8000.0, 8000.0, 8000.0 };
     
 const double dgain[] = {
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0 };

 class SR1MinimumController : public SimpleController
 {
     BodyPtr ioBody;
     double dt;
     std::vector<double> qref;
     std::vector<double> qold;

 public:

     virtual bool initialize(SimpleControllerIO* io) override
     {
	 ioBody = io->body();
	 dt = io->timeStep();

         for(int i=0; i < ioBody->numJoints(); ++i){
             Link* joint = ioBody->joint(i);
             joint->setActuationMode(Link::JointTorque);
	     io->enableIO(joint);
	     qref.push_back(joint->q());
	 }
	 qold = qref;

	 return true;
     }

     virtual bool control() override
     {
	 for(int i=0; i < ioBody->numJoints(); ++i){
	     Link* joint = ioBody->joint(i);
	     double q = joint->q();
	     double dq = (q - qold[i]) / dt;
	     double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
	     qold[i] = q;
	     joint->u() = u;
	 }
	 return true;
     }
 };

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

This controller is a sample included with Choreonoid and is built together with the Choreonoid main body by default. (It's OK if **BUILD_SIMPLE_CONTROLLER_SAMPLES** is set to ON in the CMake configuration.)

For information on how to implement and build a new simple controller separately from the samples, please refer to :doc:`howto-build-controller`.

.. _simulation-implement-controller-simple-controller-class:

SimpleController Class
----------------------

Controllers in the Simple Controller format are implemented by inheriting from the SimpleController class. This class is ::

 #include <cnoid/SimpleController>

and can be used by including the cnoid/SimpleController header.

This class basically has the following definition: ::

 class SimpleController
 {
 public:
     virtual bool initialize(SimpleControllerIO* io);
     virtual bool control();
};


By overriding the virtual functions of this class in the derived class, you describe the controller's processing content. The content of each function is as follows:

* **virtual bool initialize(SimpleControllerIO\* io)**

 Performs controller initialization. You can obtain objects and information related to control through the argument io.

* **virtual bool control()**

 Performs the controller's input, control, and output processing. During control, this function will be executed repeatedly as a control loop.

After defining a class that inherits from SimpleController, you need to define its factory function. This can be written using a macro as follows: ::

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

This allows the shared (dynamic link) library file compiled from this source to be used as an actual controller from the Simple Controller item.

.. note:: For details about the SimpleController class, please refer to "src/SimpleControllerPlugin/library/SimpleController.h" which defines this class in the source archive. Also refer to :ref:`simulation-implement-controller-simple-controller-class-supplement` at the end of this section.

.. _simulator-simple-controller-io:
.. _simulation-implement-controller-simple-controller-io:

IO Object
---------

The SimpleControllerIO type object passed as the argument io to the initialize function above is an object that handles information necessary for input/output between the controller and robot. We will call this object the "IO object" below.

This class inherits from ControllerIO. The functions defined in the ControllerIO class include the following, which can be used for controller implementation:

* **std::string controllerName() const**

 Returns the controller name.

* **Body\* body()**

 Returns the Body object for input/output.

* **std::string optionString() const**

 Returns the option string given to the controller.

* **std::vector<std::string> options() const**

 Returns the option string split by spaces.

* **std::ostream& os() const**

 Returns the output stream for outputting messages from the controller.

* **double timeStep() const**

 Returns the time step. The unit is seconds.

* **double currentTime() const**

 Returns the current time. The unit is seconds, with time 0 being the start of simulation.

.. _simulator-io-by-body-object:
.. _simulation-implement-controller-io-by-body-object:

Input/Output via Body Object
----------------------------

In Simple Controller, input/output is performed via a "Body object". The Body object is the internal representation in Choreonoid of the :doc:`../handling-models/bodymodel`, and is an instance of the "Body class" defined in C++. Since the Body class is a data structure for storing the robot model and its state, it can also store values related to joint angles, torques, and sensor states that are input/output targets. Therefore, in Simple Controller, input/output is performed via this Body object. The Body object for this purpose can be obtained with the body function of the IO object.

.. The Body class has various information and functions related to the model, so it is actually an over-spec data structure for just performing input/output. Simple Controller prioritizes ease of implementation and uses this, but as an input/output interface, it is common to use a data structure optimized for exchanging specific input/output elements rather than such a data structure.

Link Object
~~~~~~~~~~~

In Body objects, the individual parts (rigid bodies) that make up the model are represented as "Link class" objects, and information about joints is also included in these (see :ref:`model_structure`). Link objects can be obtained using the following functions of the Body class:

* **int numJoints() const**

 Returns the number of joints the model has.

* **Link\* joint(int id)**

 Returns the Link object corresponding to the joint number (id).
  
* **Link\* link(const std::string& name)**

 Returns the Link object with the name specified by name.
 
For the obtained Link object, you can access joint state values using the following member functions (state variables):

* **double& q()**

 Returns a reference to the joint displacement value. The unit is [rad] or [m].

* **double& q_target()**

 A member for referencing and setting the joint displacement command value. The unit is [rad] or [m].

* **double& dq()**

 Returns a reference to the joint velocity value. The unit is [rad/s] or [m/s].

* **double& dq_target()**

 A member for referencing and setting the joint velocity command value. The unit is [rad/s] or [m/s].

* **double& ddq()**

 Returns a reference to the joint acceleration value. The unit is [rad/s^2] or [m/s^2].

* **double& u()**

 A member for referencing and setting the joint torque (translational force) value. The unit is [N·m] or [N].

In Simple Controller, input/output to each joint is basically performed using the above state variables. That is, when inputting, you read the value of the corresponding variable, and when outputting, you write a value to the corresponding variable. (Since the above member functions return references to the corresponding variables, you can also assign values using the assignment operator.) However, to perform such input/output, you need to perform the :ref:`simulation-implement-controller-enable-io` described later in advance.

Which state variables are actually used as command values for actuators and which state variables are read as input depends on the actuator type and control method. Also, the state variables available for input/output vary depending on the :ref:`simulation_simulator_item` (physics engine) used.

State Variable Symbols
~~~~~~~~~~~~~~~~~~~~~~

Choreonoid defines symbols for identifying state variables that are input/output targets, and these are used to specify which state variables to use for command values and input values. The symbols are defined as elements of the StateFlag enumeration type in the Link class as follows. (They can be accessed from outside with the Link class scope resolution operator Link::.)

.. list-table:: **Link::StateFlag Enumeration Type Symbols**
 :widths: 20,50,30
 :header-rows: 1

 * - Symbol
   - Content
   - Corresponding State Variable
 * - **StateNone**
   - No applicable state.
   - 
 * - **JointEffort**
   - Torque (for revolute joints) or force (for prismatic joints) applied to the joint
   - Link::u()
 * - **JointTorque**
   - Same as JointEffort. Defined to make the description clearer for revolute joints.
   - Same as above
 * - **JointForce**
   - Same as JointEffort. Defined to make the description clearer for prismatic joints.
   - Same as above
 * - **JointDisplacement**
   - Joint displacement (joint angle or joint translation position)
   - Link::q() (current value) or Link::q_target() (command value)
 * - **JointAngle**
   - Same as JointDisplacement. Defined to make the description clearer when the corresponding displacement is a joint angle.
   - Same as above
 * - **JointVelocity**
   - Joint velocity component. Corresponds to angular velocity of revolute joints or displacement velocity of prismatic joints.
   - Link::dq() (current value) or Link::dq_target() (command value)
 * - **JointAcceleration**
   - Joint acceleration component. Corresponds to angular acceleration of revolute joints or displacement acceleration of prismatic joints.
   - Link::ddq()
 * - **LinkPosition**
   - Link position. Corresponds to the 6-DOF position and orientation of the link coordinate frame in Cartesian space.
   - Link::T()
 * - **LinkTwist**
   - Link velocity. Translational and angular velocity of the link coordinate frame.
   - Link::v() (translational velocity), Link::w() (angular velocity)
 * - **LinkAcceleration**
   - Link acceleration. Translational and angular acceleration of the link coordinate frame.
   - Link::dv() (translational acceleration), Link::dw() (angular acceleration)


Multiple elements can also be combined. In that case, list multiple symbols with the bit operator '|'. For example, by specifying ::

 JointDisplacement | JointVelocity

you can specify both joint displacement and joint velocity.

.. note:: The symbols up to Choreonoid 1.7 were in the format combining uppercase letters and underscores like "JOINT_EFFORT", but from Choreonoid 1.8, the symbols are in the above format. The old symbols can still be used for a while, but please use the new symbols in the future.

.. _simulation-implement-controller-actuation-mode:

Actuation Mode
~~~~~~~~~~~~~~

As a concept related to output from the controller to each link/joint, there is the "actuation mode". This determines which state variable to use as the control command value. The symbols of the StateFlag enumeration type above are used to specify the mode.

The modes corresponding to basic joint command values are as follows:


.. list-table:: **Basic Actuation Modes**
 :widths: 20,60,20
 :header-rows: 1

 * - Mode
   - Content
   - State Variable
 * - **StateNone**
   - No actuation. The joint will be in a free state.
   - 
 * - **JointEffort**
   - Uses the force/torque driving the joint as the command value.
   - Link::u()
 * - **JointDisplacement**
   - Uses the joint displacement as the command value.
   - Link::q_target()
 * - **JointVelocity**
   - Uses the joint angular velocity or displacement velocity as the command value.
   - Link::dq_target()

The actuation mode is referenced and set using the following functions of the Link class:

* **void setActuationMode(int mode)**

 Sets the actuation mode. The mode value is specified with Link::StateFlag symbols. It is also possible to specify multiple symbols combined as a bitwise OR.

* **int actuationMode() const**

 Returns the currently set actuation mode. The value is usually one element of Link::StateFlag, but may be a combination of multiple elements (bit set).


.. _simulation-implement-controller-enable-io:

Enabling Input/Output
~~~~~~~~~~~~~~~~~~~~~

Which state variables to input/output from the controller is set using the IO object. The SimpleControllerIO class defines the following functions for this:

* **void enableInput(Link\* link)**

 Enables input of the state quantities of the specified link (joint) to the controller. The appropriate state quantities for the actuation mode set for the link become input targets. For example, if JointEffort is set as the actuation mode, the current value of joint displacement Link::q() becomes the input target. This is necessary for performing PD control.

* **void enableInput(Link\* link, int stateFlags)**

 Enables input of the state quantities specified by stateFlags among those of the specified link (joint) to the controller. stateFlags is specified as a logical OR of Link::StateFlag symbols. Use this function when you clearly know which values you want to input.

* **void enableOutput(Link\* link)**

 Enables output to the specified link (joint). The state variable corresponding to the actuation mode set for the link becomes the output target. For example, if JointEffort is set as the actuation mode, Link::u() corresponding to joint torque/force becomes the output target.

* **void enableOutput(Link\* link, int stateFlags)**

 Enables output to the specified link (joint). The state variables to output are specified by specifying Link::StateFlag symbols in stateFlags.

* **void enableIO(Link\* link)**

 Enables input/output for the specified link. The appropriate state quantities for the actuation mode set for the link become input/output targets.

.. note:: SimpleControllerIO also defines functions such as setLinkInput, setJointInput, setLinkOutput, and setJointOutput. These were functions used in versions before Choreonoid 1.5, but from version 1.6 onwards, the above enableIO, enableInput, and enableOutput functions have been introduced as replacements for these functions, so please use those functions in the future.

   
The actually available actuation modes vary depending on the type and settings of the simulator item (≒ physics engine). Most simulator items support JOINT_EFFORT, and by combining this with JOINT_DISPLACEMENT input, it is possible to perform PD control, etc.

For the actuation mode set in the Link object, the input/output targets are usually as follows:

.. list-table::
 :widths: 50,25,25
 :header-rows: 1

 * - Actuation Mode
   - Input
   - Output
 * - JointEffort
   - Link::q()
   - Link::u()
 * - JointDisplacement_DISPLACEMENT
   - None
   - Link::q_target()
 * - JointVelocity
   - Link::q()
   - Link::dq_target()

.. note:: By specifying **LinkPosition**, it is also possible to directly target the position and orientation of links in 3D space for input/output. This will be explained later in :ref:`simulation-implement-controller-link-position`.


Initialization Process
----------------------

In the initialize function of the SimpleController-derived class, the controller is initialized.

In the sample, first ::

 ioBody = io->body();

obtains the Body object for input/output and stores it in the member variable ioBody. This allows this object to be used in other functions of the controller.

Similarly, for the time step (delta time) value needed in control calculations, ::

 dt = io->timeStep();

stores the value in a member variable called dt.

Next, the following for loop performs initialization processing for all joints of the robot. ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     ...
 }

First, inside this loop ::

 Link* joint = ioBody->joint(i);

obtains the link object corresponding to the i-th joint and sets it to the variable joint.

Then ::

 joint->setActuationMode(Link::JointTorque);

sets the actuation mode for this joint. Here, by specifying Link::JointTorque, joint torque is used as the command value. Also, ::

 io->enableIO(joint);

enables input/output for this joint. Since JointTorque is set as the actuation mode, the output is joint torque and the input is joint angle. This performs PD control.

Next ::

 qref.push_back(joint->q());

stores the joint angles in the robot's initial state in the vector variable qref. This is also used for PD control. This ends the for loop for each joint.

Next ::

 qold = qref;

initializes the vector variable qold with the same values as qref. This is a variable for referencing the joint angle from the previous step in PD control.

Finally, by returning true as the return value of the initialize function, we inform the simulator that initialization was successful.

Control Loop
------------

In SimpleController-derived classes, the control loop is described in the control function.

Similar to initialization, with the following for statement ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     Link* joint = ioBody->joint(i);
     ...
 }

control calculations are performed for all joints. The content is the processing code for each joint.

First, input the current joint angle. ::

 double q = joint->q();

Calculate the joint torque command value using PD control. First, calculate the joint angular velocity from the difference with the previous joint angle in the control loop. ::

 double dq = (q - qold[i]) / dt;

Since the control goal is to maintain the initial posture, calculate the torque command value with the initial joint angle as the target joint angle and 0 (stationary state) as the target angular velocity. ::

 double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];

The gain values for each joint are extracted from the pgain and dgain arrays set at the beginning of the source. Gain values need to be adjusted for each model, but the method is omitted here.

Save the joint angle in the qold variable for the next calculation. ::

 qold[i] = q;

Output the calculated torque command value. This controls the joint to maintain the initial joint angle. ::

 joint->u() = u;

By applying the above to all joints, the posture of the entire robot is also maintained.

Finally, by having this control function return true, we inform the simulator that control is continuing. This causes the control function to be called repeatedly.

.. _simulation-device:

Input/Output for Devices
------------------------

What are Devices?
~~~~~~~~~~~~~~~~~

So far, we have dealt with input/output for joint-related state quantities such as joint angles and joint torques. On the other hand, there are also input/output elements independent of joints. In Choreonoid, these are defined as "devices" and become components of the Body model.

.. In the above example, we input joint angles and output joint torques. This can be thought of as performing input/output for devices such as encoders and actuators attached to joints.

.. There can be various other devices that are input/output targets. For example, as sensors that are mainly input targets like encoders,

.. Generally, robots have various devices besides joint encoders and actuators.

Examples of devices include:

* Force sensors, acceleration sensors, angular velocity sensors (rate gyros)
* Cameras, laser range sensors

These are devices that are mainly input targets as sensors.

.. However, there may be cases where you want to output operation commands such as changing camera zoom.
.. As mainly output targets,

Also, as devices that mainly act on the external world as output targets:

* Lights
* Speakers
* Displays

(Speakers and displays are only mentioned as examples and have not been implemented yet.)

In actual controller development, you will need to perform input/output for these various devices. To do this, you need to understand:

* How devices are defined in the model
* How to access specific devices in the controller format you are using

.. _simulation-device-object:

Device Objects
~~~~~~~~~~~~~~

In Choreonoid's body model, device information is represented as "Device objects". These are instances of types that inherit from the "Device class", with corresponding types defined for each type of device. The main device types defined by default are as follows:

.. code-block:: text

 + Device
   + ForceSensor
   + RateGyroSensor
   + AccelerationSensor
   + Imu (Inertial Measurement Unit: combines the functions of acceleration and angular velocity sensors)
   + Camera
     + RangeCamera (Camera + distance image sensor)
   + RangeSensor
   + Light
     + PointLight
     + SpotLight

※ IMU becomes "Imu" as the class name in C++ source code.

Information about devices mounted on robots is usually described in model files. In standard format model files, you describe :ref:`body-file-reference-devices` in :doc:`../handling-models/modelfile/yaml-reference`.

In Simple Controller, similar to Body and Link objects, input/output for devices is performed using Device objects, which are Choreonoid's internal representation.

The device objects that the SR1 model used in this section has are as follows:

.. tabularcolumns:: |p{3.5cm}|p{3.5cm}|p{6.0}|

.. list-table::
 :widths: 30,30,40
 :header-rows: 1

 * - Name
   - Device Type
   - Content
 * - WaistAccelSensor
   - AccelerationSensor
   - Acceleration sensor mounted on waist link
 * - WaistGyro
   - RateGyroSensor
   - Gyro mounted on waist link
 * - WaistIMU
   - Imu
   - Inertial measurement unit mounted on waist link
 * - LeftCamera
   - RangeCamera
   - Distance image sensor corresponding to left eye
 * - RightCamera
   - RangeCamera
   - Distance image sensor corresponding to right eye
 * - LeftAnkleForceSensor
   - ForceSensor
   - Force sensor mounted on left ankle
 * - RightAnkleForceSensor
   - ForceSensor
   - Force sensor mounted on right ankle

.. _simulation-obtain-device-object:

Obtaining Device Objects
~~~~~~~~~~~~~~~~~~~~~~~~

Device objects can be obtained from Body objects using the following member functions:

* **int numDevices() const**

 Returns the number of devices.

* **Device\* device(int i) const**

 Returns the i-th device. The order of devices is the order of description in the model file.

* **const DeviceList<>& devices() const**

 Returns a list of all devices.

* **template<class DeviceType> DeviceList<DeviceType> devices() const**

 Returns a list of devices of the specified type.

* **template<class DeviceType> DeviceType\* findDevice(const std::string& name) const**

 Returns a device with the specified type and name if it exists.

To obtain devices of a specific type, use the template class DeviceList. DeviceList is an array that stores device objects of the specified type, and you can extract only the corresponding type from a DeviceList containing other types using its constructor or extraction operator (<<). For example, to obtain the force sensors owned by Body object "ioBody", you can do ::

 DeviceList<ForceSensor> forceSensors(ioBody->devices());

or add to an existing list with ::

 forceSensors << ioBody->devices();

DeviceList has the same functions and operators as std::vector, so you can access each object like ::

 for(size_t i=0; i < forceSensors.size(); ++i){
     ForceSensor* forceSensor = forceSensor[i];
     ...
 }

You can also use the findDevice function to identify and obtain a device by type and name. For example, the SR1 model has an acceleration sensor named "WaistAccelSensor" mounted on the waist link. To obtain this, you can do ::

 AccelerationSensor* accelSensor =
     ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");

for the Body object.

.. _simulation-implement-controller-device-io:

Device Input/Output Methods
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Input/output via Device objects is performed as follows:

* **Input**

 Execute the function

 * **void enableInput(Device\* device)**

 on the Simple Controller's IO object to enable input to the device. Then, obtain values using member functions of the corresponding Device object.

* **Output**

 After setting values using member functions of the corresponding Device object, execute the function

 * **void notifyStateChange()**

 on the Device object to notify the simulator of the device state update.

To do these, you need to know the class definition of the device you are using. For example, regarding the "AccelerationSensor" class for acceleration sensors, there is a member function "dv()" for accessing its state. This returns acceleration as a Vector3 type 3D vector.

Input from the SR1 model's acceleration sensor follows this flow. First, in the controller's initialize function ::

 AccelerationSensor* accelSensor =
     ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
 io->enableInput(accelSensor);

enable input to accelSensor. Then, at the point in the control function where you want to reference the acceleration sensor value ::

 Vector3 dv = waistAccelSensor->dv();

you can obtain it in this form.

Similarly, for ForceSensor, RateGyroSensor, and Imu, you can perform state input using the corresponding member functions.

When using visual sensors such as cameras and range sensors, preparation is required. This is explained in :doc:`vision-simulation`.

For output to devices, please refer to the "TankJoystickLight.cnoid" sample that turns lights on and off.

.. Instead of making it stand, make it a falling simulation and create a sample that displays only when acceleration exceeds a certain value?


.. * **void enableInput(Device\* device)**
..
.. Enables input of the state and data of the device specified by device to the controller.

.. _simulation-implement-controller-link-position:

Link Position and Orientation Input/Output
------------------------------------------

Another target for controller input/output is the position and orientation of links. The position and orientation referred to here is not joint angles, but the position and orientation of the link as a rigid body itself in global coordinates. This value cannot normally be input/output for actual robots. It is difficult to know the exact position and orientation of a link for a robot not fixed in space (without a very high-performance motion capture system), and it is physically impossible to directly change the position and orientation of a link through controller output. However, since such things are possible in simulation, this value input/output function is provided assuming use limited to simulation.

To do this, specify **LinkPosition** as the state quantity symbol. For output, specify **Link::LinkPosition** in the Link object's setActuationMode function and enable output using the IO object's enableIO or enableOutput functions. For input, similarly specify **Link::LinkPosition** in the IO object's enableInput function.

In Link objects, the position and orientation is stored as an Isometry3 type value. This is a customized version of the "Transform" type from Eigen, a matrix and vector library used in Choreonoid's implementation, and is basically a 3D homogeneous coordinate transformation matrix. This value can be accessed using the following functions of the Link class:

* **Isometry3& T(), Isometry3& position()**

 Returns a reference to the Isometry3 value corresponding to position and orientation.

* **Isometry3::TranslationPart translation()**

 Returns the 3D vector corresponding to the position component.

* **void setTranslation(const Eigen::MatrixBase<Derived>& p)**
   
 Sets the position component. The argument can be any type equivalent to Eigen's 3D vector.

* **Isometry3::LinearPart rotation()**

 Returns the 3x3 matrix corresponding to the orientation (rotation) component.

* **setRotation(const Eigen::MatrixBase<Derived>& R)**

 Sets the orientation (rotation) component. The argument can be any type equivalent to Eigen's 3x3 matrix.

* **setRotation(const Eigen::AngleAxis<T>& a)**

 Sets the orientation (rotation) component. The argument is Eigen's AngleAxis type that represents rotation by a rotation axis and angle.

As an example, to input the position of the root link, first in the controller's initialize function ::

 io->enableInput(io->body()->rootLink(), LinkPosition);

etc. Then in the control function ::

 Position T = io->body()->rootLink()->position();
 Vector3 p = T.translation();
 Matrix3 R = T.rotation();

etc., you can obtain the position and orientation of the root link.

Output of link position and orientation requires a simulator that supports it and is a special usage pattern. For example, with the AIST simulator item, when you set the "dynamics mode" to "kinematics", it becomes a mode that only reproduces the given position and orientation without performing dynamics calculations in the simulation. In this case, by outputting the position and orientation of the robot's root link, the root link moves to that position and orientation. Also, if you output joint angles as well, the posture resulting from forward kinematics from the root link is reproduced.

.. note:: In versions before Choreonoid 1.7, "Position" was used as the type name for storing position and orientation. The content of this type is almost the same as the above Isometry3 and can be converted to each other, but please use Isometry3 in the future.

.. _simulation-implement-controller-link-velocity-and-acceleration:

Link Velocity and Acceleration Input
------------------------------------

As state quantities of a link as a rigid body, its velocity and acceleration can also be obtained. In that case, use **LinkTwist** and **LinkAcceleration** as state quantity symbols respectively. Input of these state quantities is the same as LinkPosition. For example, assuming a link object obtained from an IO object is stored in the variable link, to obtain velocity, first in the controller's initialize function::

 io->enableInput(link, LinkTwist);

and in the control function ::

 Vector3 dv = link->v(); // Translational velocity
 Vector3 dw = link->w(); // Angular velocity

you can obtain them in this form.
These are velocity and angular velocity in global coordinates respectively.

In the case of acceleration, set with the state quantity symbol LinkAcceleration and use the link's state quantity functions dv() and dw() to obtain acceleration and angular acceleration.

Of course, it is also possible to input these state quantities simultaneously. For example ::

 io->enableInput(link, LinkPosition | LinkTwist | LinkAcceleration);

allows you to obtain all state quantities of position, velocity, and acceleration for the link object link.

.. note:: Link angular velocity and translational acceleration can also be obtained with rate gyros and acceleration sensors respectively. In actual robots, it is normal to only be able to obtain state quantities through those sensors. The method shown in this section is a simulator-specific state quantity acquisition method.

  To use those sensors, first define the sensors in the model, then perform input from the sensors following the procedure shown in :ref:`simulation-implement-controller-device-io`. The class name for rate gyro is "RateGyroSensor" and for acceleration sensor is "AccelerationSensor". Note that the coordinate system of state quantities obtained from sensors is the sensor's local coordinates. Also, for acceleration sensors, the value includes the gravity acceleration component.

.. _simulation-implement-controller-simple-controller-class-supplement:

Supplement: About SimpleController Class Definition
---------------------------------------------------

In :ref:`simulation-implement-controller-simple-controller-class`, we introduced initialize and control as virtual functions of this class, but SimpleController has other virtual functions shown below that can be overridden to describe processing.

* **virtual bool configure(SimpleControllerConfig\* config)**

 This is also a function for initializing the controller, but the execution timing differs from the initialize function. The initialize function is executed at the timing of starting simulation, but this function is executed before that, when the controller is introduced to the project (incorporated into the item tree) and associated with a specific model. If there is processing you want to execute before starting simulation, describe it here. You can obtain information related to initialization through the argument config.

* **virtual bool start()**

 This is also a function for initializing the controller, but it is executed after the initialize function in terms of timing. It is executed when the overall simulation initialization including the initialize function is completed and the controller starts operating.

* **virtual void stop()**

 This function is executed when the simulation stops.

* **virtual void unconfigure()**

 This pairs with configure and is executed when the controller becomes invalid, such as when it is deleted or disconnected from the target model.

The config object given as an argument to the configure function is similar to :ref:`simulation-implement-controller-simple-controller-io` and has the function

* **Body\* body()**

for obtaining information about the target model, and has the same members as the IO object.

However, please note that the body object obtained from config is different from that obtained from the IO object. What is obtained from the IO object is for input/output with the Body object during simulation and is generated at simulation execution time. On the other hand, what is obtained from config is the Body object originally owned by the Body item and exists before starting simulation.


Other Samples
-------------
 
Choreonoid provides various controller samples besides SR1MinimumController. Projects using these are listed in :ref:`basics_sample_project`, so please refer to them.