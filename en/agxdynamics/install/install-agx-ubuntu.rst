==========================================
Installation of AGX Dynamics (Ubuntu Linux)
==========================================

.. contents::
   :local:
   :depth: 1

This section explains how to install AGX Dynamics for Ubuntu.

Package Installation
--------------------

Please download the deb package from the AGX Dynamics download site.
Here we introduce an example of installing AGX Dynamics version 2.30.4.0 on Ubuntu 20.04.

First, install the downloaded deb file using the dpkg command.

.. code-block:: sh

 dpkg -i agx-2.30.4.0-amd64-ubuntu_20.04.deb

However, the dpkg command does not resolve dependencies on other packages. To resolve dependencies, you can use the following methods:

1. Double-click the deb file from the file manager and install using the displayed GUI tool
2. Install using the gdebi command

Method 1 is convenient, but some deb files may not be installable this way.

For method 2, do the following:

.. code-block:: sh

 sudo apt install gdebi-core
 sudo gdebi agx-2.30.4.0-amd64-ubuntu_20.04.deb

Upon successful installation, it will be installed in the /opt/Algoryx/AGX-<version> directory.

License Installation
--------------------

Next, place the AGX execution license file (agx.lic) in the installation directory to enable execution of AGX Dynamics.

Loading Environment Settings
----------------------------

Finally, load the AGX environment variable settings.
This can be achieved by executing "setup_env.bash" in the AGX directory.

Usually, add the following line to the .bashrc file in your home directory:

.. code-block:: sh

 source /opt/Algoryx/AGX-2.30.4.0/setup_env.bash

Replace the directory "AGX-2.30.4.0" part with the version of AGX you are actually using.

With this setting, terminals launched thereafter will have the AGX environment settings applied.

.. _agxdynamics-plugin-install-ubuntu-library-reference-resolution-problem:

AGX Shared Library Reference Resolution Issues
----------------------------------------------

In recent AGX versions (confirmed with 2.30.4.0), unlike previous versions, it appears that the path to AGX shared libraries is no longer set by the above script. Without modification, the references to AGX shared libraries cannot be resolved, and the AGX Dynamics plugin may fail to load when running Choreonoid.

Shared library paths can be set using the LD_LIBRARY_PATH environment variable.
To add AGX shared libraries to the path, set this environment variable as follows:

.. code-block:: sh

 export LD_LIBRARY_PATH=/opt/Algoryx/AGX-2.30.4.0/lib:$LD_LIBRARY_PATH

However, from Choreonoid development version commit b71e314d098eb24e0beaf571f6ae0fe9fdb618a2 onwards, this can also be resolved through Choreonoid build settings without setting this environment variable. This may be preferable as it has less impact on other systems. That method is explained in the next section.