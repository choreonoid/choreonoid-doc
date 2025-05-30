Building and Installing AGX Dynamics Plugin (Ubuntu Linux)
==========================================================

.. highlight:: sh

Enabling AGX Dynamics Plugin with CMake Options
-----------------------------------------------

The AGX Dynamics plugin is included in the Choreonoid source code.
To enable it, set the following options to **ON** in the :ref:`build-ubuntu-cmake` when building Choreonoid:

* **BUILD_AGX_DYNAMICS_PLUGIN**      : AGX Dynamics Plugin - AGX Dynamics simulation plugin
* **BUILD_AGX_BODYEXTENSION_PLUGIN** : AGX Body Extension Plugin - Dedicated model plugin (wires, etc.)

You can specify these as cmake command options::

 cmake -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON

or you can switch these option values to ON in the menu displayed by the ccmake command.

When you build Choreonoid with this CMake configuration, the AGX Dynamics plugin will be built simultaneously and become available for use.

.. note:: Since the AGX Body Extension Plugin depends on the AGX Dynamics Plugin, it will not be displayed in ccmake unless BUILD_AGX_DYNAMICS_PLUGIN is ON. Try turning BUILD_AGX_DYNAMICS_PLUGIN ON and running configure once.

.. note:: When you run configure in ccmake, the AGX Dynamics path AGX_DIR is automatically set, but if it is not set, please set it manually. The default path is /opt/Algoryx/AGX-<version>.

.. _agxdynamics-plugin-build-ubuntu-option-for-library-reference-resolution:

Option for AGX Dynamics Shared Library Reference Resolution
----------------------------------------------------------

The :ref:`agxdynamics-plugin-install-ubuntu-library-reference-resolution-problem` can also be resolved using CMake options when building Choreonoid.

To do this, set the CMake option **ENABLE_INSTALL_RPATH_USE_LINK_PATH** to **ON**.

In this case, the cmake command option setting would be as follows::

 cmake -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DENABLE_INSTALL_RPATH_USE_LINK_PATH=ON

For the ccmake command, first press the "T" key to switch to advanced mode.
Then navigate through the menu and set the ENABLE_INSTALL_RPATH_USE_LINK_PATH item to ON.

When you build with this setting, the path to the AGX Dynamics shared libraries that are dynamically linked will be embedded in the AGX Dynamics plugin shared library file. This is achieved using the "RPATH" feature of shared libraries. Plugin files generated this way will use the embedded paths to dependent libraries for library reference resolution. In this case, even if the corresponding libraries do not exist in the OS shared library path, they can be linked and executed at runtime.

When using the AGX Dynamics plugin, it is usually recommended to build with this option set to ON.