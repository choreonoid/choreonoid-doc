Installing ROS
=================

.. contents::
   :local:

.. highlight:: sh

Install a ROS distribution
--------------------------

If you have not installed a ROS distribution yet, follow the instructions in `ROS.org <http://wiki.ros.org>`_ - `ROS/Installation <http://wiki.ros.org/ROS/Installation>`_ .

As for the ROS version, we have confirmed that it works with Noetic Ninjemys (Ubuntu 20.04), Melodic Morenia (Ubuntu 18.04), and Kinetic Kame (Ubuntu 16.04).

You can install the ROS environment by using the following commands.

**For Ubuntu 20.04 (ROS Noetic Ninjemys)** ::

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  sudo apt update
  sudo apt install ros-noetic-desktop-full
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc

**For Ubuntu 18.04 (ROS Melodic Morenia)** ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
 sudo apt update
 sudo apt install ros-melodic-desktop-full
 echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 sudo apt install python-rosdep
 sudo rosdep init
 rosdep update

**For Ubuntu 16.04 (ROS Kinetic Kame)** ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
 sudo apt-get update
 sudo apt-get install ros-kinetic-desktop-full
 echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 sudo apt install python-rosdep
 sudo rosdep init
 rosdep update

.. note:: The above source command is to reflect the contents of setup.bash to the current shell, and is required when you continue to work in the same shell right after the installation (above settings). If you want to run the shell again after the installation, you do not need to use this command because the above settings will be reflected in the setup.bash.

.. note:: The keys obtained by the apt-key command usually have an expiration date, and when it expires, you will not be able to access the repository. In this case, run the above apt-key command again to renew the key, and you will be able to access the repository.

Install Catkin Tools
--------------------

If you want to use Choreonoid in the ROS environment, you need to use a newer version of the build tool Catkin ( `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ , also known as Catkin Tools).

On Ubuntu 20.04, Catkin Tools can be used by installing the necessary packages as follows. ::

 sudo apt install python-3-osrf-pycommon python-3-catkin-tools


For Ubuntu 18.04 and 16.04, the Python environment is different, so the installation will be done with the following command ::

 sudo apt install python-catkin-tools

Installation of rosdep on Ubuntu 20.04 (ROS Noetic Ninjemys)
------------------------------------------------------------

For "rosdep", which is a tool to resolve dependencies of ROS packages, the official wiki describes how to install it for ROS Melodic or earlier, but since Noetic, this description has been omitted from the official wiki installation page. The above installation command is also taken from the official wiki, so it will not install rosdep as it is. The rosdep package may have been handled differently from Noetic, but the details are unknown.

For this matter, it seems that it is possible to install rosdep with the following command. ::

 sudo apt install python-3-rosdep

After installation, run the following as you would with any other distribution. ::

 sudo rosdep init
 rosdep update

There is also another rosdep package called "python3-rosdep2". This seems to be a newer version, but there is not much information about its use, and it is not clear to work well with existing packages. Therefore it is probably better to use python3-rosdep.
