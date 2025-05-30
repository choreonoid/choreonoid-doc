Building and Installing AGX Dynamics Plugin (Windows)
-----------------------------------------------------

The AGX Dynamics plugin is included in the Choreonoid source code.
You can build it by turning **ON** the following options in the CMake configuration before building Choreonoid.

* **BUILD_AGX_DYNAMICS_PLUGIN**

  * AGX Dynamics Plugin - AGX Dynamics simulation plugin

* **BUILD_AGX_BODYEXTENSION_PLUGIN**

  * AGX Body Extension Plugin - Dedicated model plugin (wires, etc.)

When configuring with CMake GUI, check **BUILD_AGX_DYNAMICS_PLUGIN** and press Configure.
At this time, the AGX library will be automatically detected. If it is not detected, set the installation directory in **AGX_DIR**.
Next, check **BUILD_AGX_BODYEXTENSION_PLUGIN** and press Configure again.

.. note:: Since the AGX Body Extension Plugin depends on the AGX Dynamics Plugin, it will not be displayed in CMake unless BUILD_AGX_DYNAMICS_PLUGIN is ON. Try turning BUILD_AGX_DYNAMICS_PLUGIN ON and running configure once.

Press Generate to generate the solution file. After that, build and install as described in :ref:`build-windows-visualstudio`. This will build and install the AGX Dynamics Plugin and AGX Body Extension Plugin, making them available for use in Choreonoid.

.. note:: When you perform the installation, the AGX Dynamics runtime libraries (DLL files) will also be installed to Choreonoid's bin directory. If you do not want to do this, turn off the **INSTALL_AGX_RUNTIME** option in the CMake settings. However, in that case, you need to set the PATH to the AGX Dynamics bin directory in advance so that the AGX Dynamics DLLs can be loaded.