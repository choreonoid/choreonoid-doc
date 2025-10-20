Installation via Package (Ubuntu Linux)
========================================

Starting from version 2.3.0, Choreonoid provides deb packages for Ubuntu Linux that can be installed using APT.
This is the easiest method for installing on Ubuntu Linux. This document explains how to use this method.

.. contents::
   :local:

.. highlight:: sh

Available APT Repositories
--------------------------

The officially provided Choreonoid deb packages are published in `Launchpad <https://launchpad.net/>`_ `PPA <https://launchpad.net/~launchpad/+archive/ubuntu/ppa>`_ repositories.
There are two types: development version and release version, with the following repositories:

* Development version

  * ppa:choreonoid.org/testing

* Release version

  * ppa:choreonoid.org/stable

The development version is a packaged version of the master branch from `Choreonoid's official Github repository <https://github.com/choreonoid/choreonoid>`_, providing the latest version of Choreonoid including features under development.
Since Choreonoid is still under active development, we recommend using this development version unless you have a specific reason not to.
Note that currently, packages are not updated with every master branch update, but are updated at appropriate intervals when master branch updates accumulate to some extent.

The release version consists of releases with version numbers assigned at development milestones.
This version should be used when you need to use a specific version or when issues occur with the development version.

Registering the APT Repository
-------------------------------

To install Choreonoid packages from the above repositories, you need to register the repository in advance.
To use the development version, register the repository with the following command: ::

 sudo add-apt-repository ppa:choreonoid.org/testing

To use the release version, use the following command: ::

 sudo add-apt-repository ppa:choreonoid.org/stable

Installing Choreonoid
---------------------

Once the above repository is registered, you can install the "choreonoid" package using the apt command.
Specifically, execute the following commands: ::

 sudo apt update
 sudo apt install choreonoid

These commands will install the latest version from the registered repository.
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

 sudo add-apt-repository --remove repository_name

The repository name is ppa:choreonoid.org/testing for the development version, or ppa:choreonoid.org/stable for the release version.

If you have another Choreonoid binary on the same PC installed by :doc:`building from source code <build-ubuntu>` or other means, conflicts may occur. To operate normally, you need to configure to avoid conflicts, so please be careful. If you're unsure about this, it's safer to uninstall the package-installed Choreonoid when building and installing from source code.

Supplement: Specifying the Version to Install
----------------------------------------------

When installing with apt, the latest version registered in the repository is basically installed, but if you want to install an older version, do the following.

First, check the available versions with the following command: ::

 apt-cache policy choreonoid

For example, suppose this command produces the following output:

.. code-block:: text

   choreonoid:
     Installed: (none)
     Candidate: 2.4.0~git20251020.1758.4e4a671b7-1~noble
     Version table:
        2.4.0~git20251020.1758.4e4a671b7-1~noble 500
           500 https://ppa.launchpadcontent.net/choreonoid.org/testing/ubuntu noble/main amd64 Packages
        2.3.0-1~noble 500
           500 https://ppa.launchpadcontent.net/choreonoid.org/stable/ubuntu noble/main amd64 Packages

Here, "2.4.0~git20251020.1758.4e4a671b7-1~noble" in the "Version table" is the detailed version name.
By specifying this version name as: ::

  sudo apt install choreonoid=2.4.0~git20251020.1758.4e4a671b7-1~noble

you can install a specific version.
