Installation via Package (Ubuntu Linux)
========================================

Starting from version 2.3.0, Choreonoid provides deb packages for Ubuntu Linux that can be installed using APT.
This is the easiest method for installing on Ubuntu Linux. This document explains how to use this method.

.. contents::
   :local:

.. highlight:: sh

Adding the APT Repository
-------------------------

Choreonoid's deb packages are published in a Launchpad PPA repository.
First, enable this repository with the following command: ::

 sudo add-apt-repository ppa:choreonoid.org/stable

Installation
------------

Install with the following commands: ::

 sudo apt update
 sudo apt install choreonoid

These commands will install the latest version.
If already installed, it will upgrade to a newer version if available.

Starting Choreonoid
-------------------

You can start Choreonoid with the following command: ::

  choreonoid

Installed Contents
------------------

Currently, there is only one package "choreonoid", which installs Choreonoid itself with most of its bundled plugins and samples enabled. Development files (CMake files, header files, library files, etc.) are also installed, allowing you to develop Choreonoid plugins and controllers.

For the actual enabled plugins, please check the message view when Choreonoid starts.

.. note:: While it is common for deb packages to separate the software itself, plugins, and development files into different packages, Choreonoid currently does not separate packages this way, and everything is included in a single "choreonoid" package.

Uninstallation
--------------

You can uninstall with the following command: ::

 sudo apt remove choreonoid

If you want to unregister the repository, execute: ::

 sudo add-apt-repository -r ppa:choreonoid.org/stable

If you have another Choreonoid binary on the same PC installed by :doc:`building from source code <build-ubuntu>` or other means, conflicts may occur. To operate normally, you need to configure to avoid conflicts, so please be careful. If you're unsure about this, it's safer to uninstall the package-installed Choreonoid when building and installing from source code.