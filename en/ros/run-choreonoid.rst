Running Choreonoid in the ROS environment
=========================================

.. contents::
   :local:

.. highlight:: sh

.. _choreonoid_ros_run_ros_master:

Starting the ROS master
-----------------------

If the ROS master has not been started, start it. Usually, open a new terminal for this purpose and execute the following command to start the ROS master there. ::

 roscore

.. _choreonoid_ros_run_choreonoid_node:

Starting Choreonoid
-------------------

In the ROS environment, Choreonoid is usually treated as a ROS node. This is called a Choreonoid node.

The command to invoke the Choreonoid node is included in the choreonoid_ros package, and the command name is choreonoid.
Thus, for example, you can use the rosrun command to invoke a Choreonoid node as follows. ::

 rosrun choreonoid_ros choreonoid

A terminal to run this command should also be prepared separately.

The main window of Choreonoid appears when it is successfully started. This is basically the same as when Choreonoid is started in the usual way, and the operation is also the same.

.. note:: By starting Choreonoid as a ROS node in this way, the ROS-related initialization and other operations are performed inside Choreonoid, and the ROS functions can be used on Choreonoid. Please note that if you run the choreonoid executable file as it is as a normal startup method, such ROS-related initialization will not be performed and you will not be able to use the ROS functions.

When the node is started, options related to ROS remapping can be given as in other ROS nodes. The options for Choreonoid itself can also be given.

Note that if the ROS master is not running, the node stops with the following error message displayed on the console because it cannot initialize ROS-related operations.

.. code-block:: none

 Choreonoid's ROS node cannot be invoked because the ROS master is not found.

In this case, please try to start the ROS master first.

Running a sample project
------------------------

Let's run the "ROS-Tank" project included in the choreonoid_ros_samples package as a sample project.

If the choreonoid_ros_samples package has been installed, you can run the sample by the following command. ::

 roslaunch choreonoid_ros_samples tank.launch

This sample uses the ROS communication functions to manually operate a tank-like "Tank" robot model using a gamepad.
This sample is a minimal implementation of this, with a simple controller that subscribes to the ROS joy topic to control the Tank model.

In fact, **you need to have a gamepad (joystick) connected to your PC in order to run this sample.**
Most gamepads that can be connected via USB can be used, so please connect the gamepad to the PC first, and then execute the above command. Once the project is launched, the simulation will start and you can move the Tank by manipulating the axes of the gamepad. If this sample works, the installation of Choreonoid in the ROS environment has been successful.

Tip: What to do if the execution of roslaunch fails?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When you try to run the above roslaunch command, the following error message may be shown and the program may not run properly.

.. code-block:: none

 RLException: [tank.launch] is neither a launch file in package [choreonoid_ros_samples] nor is [choreonoid_ros_samples] a launch file name
 The traceback for the exception was written to the log file

In this case, running :ref:`loading_catkin_workspace_setup_script` again may help.
So you can execute ::

 source ~/catkin_ws/devel/setup.bash

on your terminal, or if this script is already in ~/.bashrc, start a new terminal and operate it there.
This will update the information in the workspace, and the launch file may be recognized and ready to run.

Apparently, in the ROS (Catkin) workspace, when a new package or a new element of a package is added, the contents loaded by setup.bash may be updated. And if the update is not fetched in the terminal, the operations for such additional elements may not work. You may need to be careful about this when introducing new packages or developing your own packages.

Tip: About gamepad mapping
~~~~~~~~~~~~~~~~~~~~~~~~~~

The mapping of the gamepad axes and buttons varies from manufacturer to manufacturer and model to model,
and some mappings may not match the operation interface of the above sample.
The above sample tries to match the mapping as much as possible for the gamepads like followings:

* `Logitech F310 <http://gaming.logicool.co.jp/ja-jp/product/f310-gamepad>`_
* `DUALSHOCK4 <http://www.jp.playstation.com/ps4/peripheral/cuhzct1j.html>`_
* DUALSHOCK3
* `Controller for Xbox <https://www.xbox.com/ja-JP/xbox-one/accessories/controllers/xbox-black-wireless-controller>`_
* Controller for Xbox360

If you use a gamepad other than these, you may not be able to control it as you wish.

What roslaunch does
~~~~~~~~~~~~~~~~~~~

This sample is achieved by using roslaunch to launch multiple ROS nodes.

.. code-block:: xml

 <launch>.
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
         args="$(find choreonoid_ros_samples)/project/ROS-Tank.cnoid --start-simulation" />
 </launch>.

This description starts the following two nodes.

* choreonoid_joy: A node that publishes the joystick (gamepad) status as a joy topic
* choreonoid: Node of Choroenoid itself

There is a ROS standard joy node that does the same thing as choreonoid_joy, but that node does not have a function to adjust the gamepad mapping. The choreonoid_joy node uses the Choreonoid library to map the gamepad axes and buttons to the Choreonoid standard mapping, and the puslish information reflects this.

As for Choreonoid itself, the project named "ROS-Tank.cnoid" contained in the project directory of choreonoid_ros_samples is loaded. Also, the "--start-simulation" option is added to start the simulation at the same time when Choreonoid is started.
