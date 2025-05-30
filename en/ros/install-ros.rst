Installing ROS
==============

.. contents::
   :local:

.. highlight:: sh

Installing a ROS Distribution
-----------------------------

If you haven't installed a ROS distribution yet, please follow the instructions at `ROS.org <http://wiki.ros.org>`_ - `ROS/Installation <http://wiki.ros.org/ROS/Installation>`_ to install one.

Regarding ROS versions, we have confirmed operation with Noetic Ninjemys (Ubuntu 20.04).

You can install the ROS environment with the following commands. (For the latest installation instructions, please refer to the information on ROS.org mentioned above.)

.. http://wiki.ros.org/noetic/Installation/Ubuntu

**For Ubuntu 20.04 (ROS Noetic Ninjemys)** ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt install curl # if you haven't already installed curl
 curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
 sudo apt update
 sudo apt install ros-noetic-desktop-full
 echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
 sudo rosdep init
 rosdep update

.. note:: The source command above is used to apply the contents of setup.bash to the current shell. This is necessary when you want to continue working in the same shell immediately after installation (with the above settings). If you start a new shell after installation, this command is not necessary as the contents of setup.bash will be applied automatically through the settings above.

.. note:: After registering the ROS repository with the above settings, if a considerable amount of time passes, you may lose access to the ROS repository. In such cases, you may be unable to update OS packages due to related errors. To resolve this, try updating the repository key according to the latest information on the relevant page at ROS.org mentioned above. This may solve the problem.


Installing Catkin Tools
-----------------------

When using Choreonoid with ROS, you need to use the newer version of the Catkin build tool ( `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ , commonly known as Catkin Tools).

For Ubuntu 20.04, you can enable Catkin Tools by installing the necessary packages as follows: ::

 sudo apt install python3-osrf-pycommon python3-catkin-tools