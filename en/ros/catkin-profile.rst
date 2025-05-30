Using Catkin Profile Features
=============================

When developing ROS nodes and systems, you often need to switch between various build options. Catkin's profile feature is convenient for this purpose. This section introduces how to use it.

.. contents::
   :local:

.. highlight:: sh

.. _ros_catkin_profile_overview:

Profile Feature
---------------

Catkin allows you to configure various build and execution settings using the "catkin config" command. There are cases where you want to switch between several sets of these configurations. Examples include:

* Normally using release build options for fast performance, but switching to debug options when debugging
* Temporarily narrowing the build and execution targets to focus on developing specific features in the system, then returning to full system building when done
* Temporarily including experimental features (nodes), then reverting when finished

Especially when the system includes nodes written in C++, the behavior of binaries is significantly affected by build settings, so you often need to change build configurations depending on the situation.

In such situations, repeatedly specifying related settings one by one with catkin config each time you switch is time-consuming and increases the risk of configuration errors. While you could prepare separate catkin workspaces for each configuration, this requires the effort of setting up additional workspaces and unnecessarily increases storage requirements and build time.

This problem can be solved with Catkin's profile feature. The profile feature manages workspace settings with multiple "profiles". By preparing each configuration you want to switch between as a profile, you can easily switch between them. In the debugging example above, you can switch between a "release profile" and a "debug profile".

For details on this feature, refer to the `profile command page <https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_profile.html>`_ in the `Catkin Command Line Tools manual <https://catkin-tools.readthedocs.io/en/latest/index.html>`_. For usage examples, the manual's `Profile Cookbook <https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html#profile-cookbook>`_ is helpful.

.. _ros_catkin_profile_operations:

Basic Profile Operations
------------------------

Profiles basically correspond to settings made with catkin config. Settings are normally made under the profile name "default", but you can specify the target profile name. For example, to create a debug profile: ::

 catkin config --profile debug -x _debug --cmake-args -DCMAKE_BUILD_TYPE=Debug ...

Here we first provide the profile name "debug" with the "--profile" option. We also specify the suffix "_debug" with the "-x" option. This suffix is added to build-related directories. For example, while builds normally occur in the "build" directory and execution targets are managed in the "devel" directory, in the above example these become "build_debug" and "devel_debug" respectively. By specifying this, you don't need to rebuild every time you switch profiles.

Then, as with normal catkin config commands, configure build settings using "--cmake-args" and similar options. Here we set the CMake build type to "Debug" to build with debugging options. You can make any other settings as needed.

You can also create profiles with: ::

 catkin profile add profile_name

.. note:: According to the add command help, you should be able to copy settings from existing profiles using the "--copy" or "--copy-active" options. However, when tested with catkin tools version 0.5.0, these options prevented profile creation, and even when specified with the "--force" option on existing profiles, the settings weren't copied properly. This might be a bug in this version of catkin.

You can check profiles managed in the workspace with: ::

 catkin profile list

This command produces output like: ::

 [profile] Available profiles:
 - default (active)
 - debug

Here you can confirm that a debug profile exists in addition to the default profile. You can also see that the default profile is active.

If you don't specify a profile in catkin commands, the currently active profile is used. You can switch the active profile with the "catkin profile set" command. To make the debug profile active: ::

 catkin profile set debug

You can also specify the target profile for each catkin command without changing the active profile. Use the "--profile" option for this. The catkin config example shown earlier used this. This option works with other catkin commands, so you can apply it to catkin build like: ::

 catkin build --profile debug

This performs a build based on the debug profile settings. Of course, if you switch the active profile to debug beforehand, the build targets the debug profile without this option.


Running Outputs from Specific Profiles
--------------------------------------

While you can switch profiles for configuration and building as described above, you need to be careful when running outputs from switched profiles. For example, as we've assumed, suppose the default profile is for release builds and there's a separate debug profile for debugging. In this situation, when running Choreonoid with: ::

 rosrun choreonoid_ros choreonoid

How do you switch between release and debug binaries?

If you haven't set suffixes for profiles, the directories storing build outputs in the workspace are the same. In this case, only outputs from the last build profile are available in the workspace. Therefore, when switching the binary to run, you need to rebuild to generate it. Since you basically need to rebuild everything when build options differ, this method isn't suitable if you want to frequently switch between profiles for execution. (After switching profiles, you often need to run catkin clean before catkin build.)

Differently, if you've specified different suffixes for profiles, outputs for each profile can exist simultaneously in the workspace. For example, if the debug profile has the suffix "_debug", outputs for each profile are stored in:

* Default profile (release version)
 * workspace_directory/devel
* Debug profile (debug version)
 * workspace_directory/devel_debug

.. note:: As shown in :ref:`ros_catkin_profile_operations`, you can specify suffixes with the "-x" option of the catkin config command. To set the suffix as debug, use "catkin config -x debug".

In this case, you don't need to rebuild every time you switch profiles. However, the question becomes which directory's outputs are executed at runtime.

Catkin itself doesn't seem to have a feature to switch execution target profiles. To do this, you need to change the script targeted by :ref:`loading_catkin_workspace_setup_script`. By default, we load the script "workspace_directory/devel/setup.bash". This sets environment variables like PATH so that outputs under devel are executed. To execute outputs with the "_debug" suffix, switch the loaded script to "workspace_directory/devel_debug/setup.bash".

Specifically, replace the content in .bashrc with: ::

 source $HOME/catkin_ws/devel_debug/setup.bash

Then restart the terminal or execute this command in the terminal. (However, if you execute this command in the terminal, PATH settings are added each time, which might not be ideal.)

When profiles have suffixes, you can switch execution target profiles with this method.


Integration with Debuggers
--------------------------

By preparing debug binaries using the above method, you can debug Choreonoid itself, plugins, and other C++ ROS nodes using debuggers.

Since ROS programs are basically executed with commands like rosrun or roslaunch, it might be somewhat difficult to run them directly in a debugger. However, debuggers typically have an "attach to existing process" feature, so you can first start the program and then connect the debugger to it. This method allows you to debug ROS systems.