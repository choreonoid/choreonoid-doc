Installing ROS 2
================

.. contents::
   :local:

.. highlight:: sh

Installing a ROS 2 Distribution
-------------------------------

First, you need to install a ROS 2 distribution.
For the latest and detailed installation instructions, please refer to the following official documentation:

* `ROS 2 Documentation: Jazzy - Installation <https://docs.ros.org/en/jazzy/Installation.html>`_
* `ROS 2 Documentation: Humble - Installation <https://docs.ros.org/en/humble/Installation.html>`_

.. note:: As of June 1, 2025, the official ROS 2 installation method has changed. Accordingly, the description on this page has been updated. For details on the installation method changes, please refer to the `ROS signing key migration guide <https://discourse.ros.org/t/ros-signing-key-migration-guide/43937>`_.

As of June 2025, you can install a ROS 2 distribution with the following commands:

**Common preparation for all versions**  ::

  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
  sudo apt install /tmp/ros2-apt-source.deb
  sudo apt update

Run one of the following depending on the version you want to use:

**Installing ROS 2 Jazzy on Ubuntu 24.04 LTS** ::

  sudo apt install ros-jazzy-desktop
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  source ~/.bashrc

**Installing ROS 2 Humble on Ubuntu 22.04 LTS** ::

  sudo apt install ros-humble-desktop
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc

.. note:: ROS 2 operates in a locale environment that supports UTF-8. When installing Ubuntu with Japanese language settings, it typically uses the "ja_JP.UTF-8" locale, which supports UTF-8. In minimal environments such as Docker containers, it might be set to a minimal locale like "POSIX", in which case you need to configure it to support UTF-8. For details, please refer to the installation page in the official ROS 2 documentation.

.. note:: The final source command is to apply the contents of setup.bash to the current shell. This is necessary when continuing to work in the same shell immediately after installation with the series of commands. If you start a new shell after installation, the contents of setup.bash will be applied through bashrc, so you don't need to execute this command.

.. _ros2_install_ros2_install_dev_tools:

Installing Development Tools
----------------------------

When using Choreonoid with ROS 2, you currently need to :doc:`build-choreonoid` in the ROS 2 environment.
You may also need to create control programs for robots as ROS packages.
Therefore, install the ROS 2 development tools package "ros-dev-tools" with the following command: ::

  sudo apt install ros-dev-tools

Installing this package also installs "colcon", the build system for ROS 2.

Additionally, you can enable the "colcon clean" command to clean binaries built with colcon by running: ::

  sudo apt install python3-colcon-clean

This is not strictly necessary, but it may be convenient to have.

Initializing rosdep
-------------------

rosdep is a tool for installing packages used in ROS 2 based on their dependencies.
Since you may use rosdep when using Choreonoid, let's also initialize rosdep.
rosdep itself is installed with the ros-dev-tools package mentioned above, but after installation, you need to initialize it with the following commands: ::

  sudo rosdep init
  rosdep update

※ The second command should be executed without sudo.

For details about rosdep, please refer to:

* `ROS 2 Documentation: Jazzy - Managing Dependencies with rosdep <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html>`_
* `ROS 2 Documentation: Humble - Managing Dependencies with rosdep <https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html>`_