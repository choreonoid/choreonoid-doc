Running Choreonoid in the ROS Environment
=========================================

.. contents::
   :local:

.. highlight:: sh

.. _choreonoid_ros_run_ros_master:

Starting the ROS Master
-----------------------

If the ROS master is not running, start it first. (Usually, open a new terminal for this purpose and run it there.) ::

 roscore

.. _choreonoid_ros_run_choreonoid_node:

Starting Choreonoid
-------------------

In the ROS environment, Choreonoid is typically handled as a ROS node. This is called a Choreonoid node.

The command to start the Choreonoid node is included in the choreonoid_ros package, and the command name is choreonoid. Therefore, using the rosrun command, for example: ::

 rosrun choreonoid_ros choreonoid

will start Choreonoid as a ROS node. (Prepare a separate terminal for this as well.)

Upon successful startup, Choreonoid's main window will appear. This is basically the same as when Choreonoid is started normally, and the operations are the same.

.. note:: By starting Choreonoid as a ROS node this way, ROS-related initialization is performed internally in Choreonoid, enabling the use of ROS functionality within Choreonoid. Please note that if you execute the choreonoid executable directly as a normal startup method, such ROS-related initialization will not occur, and you will not be able to use ROS functionality.

When starting the node, you can provide ROS remapping options just like with other ROS nodes.

You can also provide Choreonoid's own options.

If the ROS master is not running, ROS-related initialization cannot be performed, so the following error message will be displayed in the console and execution will stop:

.. code-block:: none

 Choreonoid's ROS node cannot be invoked because the ROS master is not found.

In this case, please start the ROS master first.


Running a Sample Project
------------------------

Let's run the "ROS-Tank" project included in choreonoid_ros_samples as a sample project.

If the choreonoid_ros_samples package is installed, you can launch the sample with the following command: ::

 roslaunch choreonoid_ros_samples tank.launch

This sample demonstrates a simulation where a tank-like "Tank" model is manually controlled using a gamepad, implemented using ROS mechanisms. This sample provides a minimal implementation that controls the Tank model through a simple controller that subscribes to the ROS joy topic.

While we've introduced how to launch the sample, **you must have a gamepad (joystick) connected to your PC to run this sample.** Most USB-connectable gamepads can be used, so please connect a gamepad to your PC before executing the above command. When the project starts, the simulation will also begin, and you can move the Tank by operating the gamepad axes. If this sample works, your installation of Choreonoid in the ROS environment is successful.


Troubleshooting: When roslaunch Fails to Execute
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When trying to execute the above roslaunch command, you might see an error message like:

.. code-block:: none

 RLException: [tank.launch] is neither a launch file in package [choreonoid_ros_samples] nor is [choreonoid_ros_samples] a launch file name
 The traceback for the exception was written to the log file

and the execution may fail.

In this case, re-executing :ref:`loading_catkin_workspace_setup_script` may resolve the issue. That is, execute: ::

 source ~/catkin_ws/devel/setup.bash

on your terminal, or if this script is already written in ~/.bashrc, open a new terminal and work there. This updates the workspace information, allowing the launch file to be recognized and executed.

It seems that in ROS (Catkin) workspaces, when new packages or new elements of packages are added, the content loaded by setup.bash may need to be updated. If these updates haven't been incorporated into the terminal, operations targeting such additions may not work properly. You need to be aware of this when introducing new packages or developing your own packages.


Note: About Gamepad Mapping
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Gamepad axis and button mappings vary by manufacturer and model, and some mappings may not match the sample's control scheme. The above sample attempts to accommodate mappings for:

* `Logitech F310 <http://gaming.logicool.co.jp/ja-jp/product/f310-gamepad>`_
* `DUALSHOCK4 <http://www.jp.playstation.com/ps4/peripheral/cuhzct1j.html>`_
* DUALSHOCK3
* `Xbox Controller <https://www.xbox.com/ja-JP/xbox-one/accessories/controllers/xbox-black-wireless-controller>`_
* Xbox 360 Controller

These gamepads will work with this functionality. Other gamepads may not operate as expected, please understand.

roslaunch Execution Details
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This sample is implemented by launching multiple ROS nodes using roslaunch. The launch file is as follows:

.. code-block:: xml

 <launch>
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
         args="$(find choreonoid_ros_samples)/project/ROS-Tank.cnoid --start-simulation" />
 </launch>

This description launches the following two nodes:

* choreonoid_joy: A node that publishes the joystick (gamepad) state as a joy topic
* choreonoid: The Choreonoid main node

While there is a standard ROS joy node that performs similar processing to choreonoid_joy, it doesn't have the functionality to match gamepad mappings. choreonoid_joy uses Choreonoid's library to match gamepad mappings to Choreonoid's standard mapping, and the published information reflects this.

For Choreonoid itself, it loads the "ROS-Tank.cnoid" project contained in the project directory of choreonoid_ros_samples. Additionally, by providing the "--start-simulation" option, the simulation starts simultaneously when Choreonoid launches.