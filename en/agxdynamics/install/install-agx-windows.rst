Installing AGX Dynamics (Windows)
=================================

.. contents::
   :local:
   :depth: 1

Installation
------------

This section explains how to install AGX Dynamics for Windows.
Download the file for **x64**, **VS2019** from the AGX Dynamics download site and execute it.
As of November 2021, we have confirmed that versions 2.29, 2.30, and 2.31 can be used.

The installer will launch, so please follow the displayed instructions.
For AGX Dynamics version 2.30 and later, it is installed by default in c:\Users\username\AppData\Local\Algoryx\AGX-<version>. In this case, it will be installed individually for each user. In version 2.29 and earlier, it was installed under c:\Program Files and was a common installation for the entire system. Both cases are supported.

At the end of the installation, you will be asked:

* Install Python libraries?
* Open AGX Dynamics Documentation and tutorials?

Both are checked by default and will be executed. However, neither is necessary when using the AGX Dynamics plugin with Choreonoid, so it's okay to uncheck them. Especially for the first Python-related item, if you check it and execute it, it will take a long time to download the corresponding libraries and will change your Python environment, so it's safer to uncheck it.

After the installer completes, copy the AGX execution license file (agx.lic) to the installation directory (such as AGX-2.31.0.2). If you have a dongle, connect it to your PC as well. If this process is not done properly, you will get a license error when executing simulations using AGX Dynamics, and you will not be able to run simulations.

Operation Verification
----------------------

Run the AGX Dynamics sample to verify operation. Open a command prompt and execute the following commands in order:

.. code-block:: txt

   cd <AGX Dynamics installation directory>
   setup_env.bat
   cd bin\x64
   tutorial_trackedVehicle.exe