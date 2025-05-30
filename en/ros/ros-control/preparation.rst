Preparation
===========

.. contents::
   :local:

.. _ros_cnoid_tank_setup:

Setup of ROS and Choreonoid Related Packages
--------------------------------------------
To carry out this tutorial, you must first complete :doc:`../install-ros` and :doc:`../build-choreonoid`. Please follow the procedures on these pages to install and build, and confirm that you can execute :doc:`../run-choreonoid`.

Note that :ref:`ros_choreonoid_add_package_sources` does not need to be executed. (Of course, there is no particular problem if it is already installed.)

.. _ros_cnoid_pull_and_build:

Downloading and Building cnoid_tank_pkgs
----------------------------------------

.. highlight:: sh

In this tutorial, the path to the catkin workspace is denoted as "<catkin_ws>", so please replace it accordingly. Note that the workspace name itself doesn't have to be "catkin_ws" (for example, "choreonoid_ws" is fine). We recommend using separate workspaces for each project.
               
First, download (git clone) the "cnoid_tank_pkgs" repository under the catkin workspace: ::

 mkdir -p <catkin_ws>/src
 cd <catkin_ws>/src
 git clone https://github.com/choreonoid/cnoid_tank_pkgs.git

Once completed, download the dependency packages.
Here, we first use `wstool <http://wiki.ros.org/wstool>`_ to download the dependency packages: ::

  cd <catkin_ws>
  wstool init src
  wstool merge -t src src/cnoid_tank_pkgs/melodic.rosinstall
  wstool up -t src

At this point, the dependency packages will be automatically downloaded under the "<catkin_ws>/src" directory.
Next, download the dependency packages provided as "apt" packages. We use `rosdep <http://wiki.ros.org/rosdep>`_ for this: ::

  cd <catkin_ws>
  rosdep update
  rosdep install -i -y -r --from-paths src

At this point, the downloading of dependency packages is complete.
Finally, let's build the packages using either `catkin_make <http://wiki.ros.org/catkin/commands/catkin_make>`_ or `catkin tools <https://catkin-tools.readthedocs.io/en/latest/>`_. Here, we'll use `catkin tools <https://catkin-tools.readthedocs.io/en/latest/>`_: ::

  cd <catkin_ws>
  catkin build

This completes the downloading and building of the packages.
Good job!