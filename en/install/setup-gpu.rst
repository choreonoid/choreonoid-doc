Graphics Environment Setup
=====================================

.. contents::
   :local:
   :depth: 1

.. highlight:: sh

About Graphics Environment
--------------------------

Choreonoid renders robot and environment models using three-dimensional computer graphics (3DCG). To accomplish this, the graphics environment of the PC being used must meet the following requirements:

1. OpenGL (version 1.5 or higher, 3.3 or higher recommended), a three-dimensional rendering API, must be available
2. GPU hardware acceleration for OpenGL must be available

Regarding requirement 1, this is usually available on :doc:`platform`. This requirement must first be met for Choreonoid to operate.

Additionally, to use Choreonoid comfortably, it is desirable that requirement 2 is also met. Otherwise, 3DCG rendering will become extremely slow, making it impractical unless you are using very simple models.

On Windows, OpenGL hardware acceleration is usually available in the state when the PC is purchased. However, please note that if you install Windows yourself or replace the video board, you may need to separately install graphics drivers for the installed GPU.

On Linux as well, open source drivers have improved recently, and OpenGL acceleration often functions immediately after installation. However, depending on the GPU, acceleration may not be sufficient as-is, so caution is required. The situation for each GPU type is described later.

When using virtual machines, OpenGL hardware acceleration often does not function sufficiently. In such cases, rendering may be slow or may not render properly. This is a limitation on the virtual machine side. Generally, such situations are common with virtual machines, so their use is not highly recommended, and if you do use them, please do so with this understanding. (However, the situation with recent virtual machines seems to have improved compared to before.)

.. _setup_gpu_recommended_gpus:

GPU Types
---------

GPU types can be broadly distinguished by manufacturer. The situation may also differ depending on the model number. Below is a brief summary of the situation for each GPU type.

* NVIDIA GPUs (GeForce/Quadro, etc.)

 * They operate without problems and have excellent performance.
 * When seeking the highest performance, NVIDIA high-end models are currently strong candidates.
 * When using with Ubuntu, proprietary drivers must be separately installed to achieve full performance. GUI tools are also provided for this purpose, so driver installation is not particularly difficult.

* AMD GPUs (Radeon)
 * Previously there were issues with using Choreonoid, but these have been improved in the latest version.
 * Previously Linux support was insufficient, but this has improved since several years ago. Specifically, GPUs from the "GCN architecture" generation onwards operate without problems on Linux.
 * The GCN architecture corresponds to specific model numbers including Radeon HD 7700-7900, HD 8000, RX 240-290, RX 300, RX 400, RX 500, and Vega series. Please avoid using older model numbers than these.
 * The above model numbers seem to operate generally without problems even with the open source drivers installed by default in Ubuntu.
 * Performance when used on Linux appears to be inferior compared to the combination of NVIDIA GPUs with proprietary drivers.
 * Proprietary drivers may also be available on Linux. This depends on the GPU chip type and kernel version, so please check AMD's website for details.

* Intel CPU integrated GPUs (HD Graphics, UHD Graphics)

 * These are GPUs integrated into Intel Core i-series and other CPUs. These also operate without problems.
 * Regarding rendering speed, they are basically not as fast as GPUs from the above two manufacturers. They should be fine when using relatively simple models.
 * They operate without problems with drivers installed by default in Ubuntu. Actually, there are no other drivers available.
 * However, older model numbers may not render shadows properly. For such model numbers, shadow rendering is disabled.

NVIDIA GPUs are recommended for use considering their track record, stability, performance, and other factors comprehensively. Regarding AMD GPUs, while we did not recommend them previously, drivers have improved recently, and they are sufficiently usable when combined with the latest development version of Choreonoid. Intel CPU integrated GPUs do not have exceptional performance but operate stably, so if your PC is equipped with one, it would be good to try it first.


Installing GPU Drivers on Windows
----------------------------------

On Windows, appropriate drivers are often installed by default, so you usually don't need to worry about this much. If three-dimensional model rendering in Choreonoid does not work properly, please check whether drivers are correctly installed.

.. _build_ubuntu_gpu_driver:
.. _setup_gpu_ubuntu_gpu_driver:

Installing GPU Drivers on Ubuntu Linux
---------------------------------------

On Linux, GPU drivers may need to be installed separately. The following explains the methods for each GPU type.

For NVIDIA GPUs
^^^^^^^^^^^^^^^

For PCs equipped with NVIDIA GPUs such as GeForce or Quadro, the driver installed by default in Ubuntu Linux is an open source version called "Nouveau". Unfortunately, this currently does not provide sufficient 3D rendering hardware acceleration, so its use is not recommended.

In contrast, proprietary drivers developed by NVIDIA can be used. Since these are developed by the manufacturer itself, their functionality and performance are excellent, and it is possible to utilize the GPU's capabilities to the fullest.

Checking the status of proprietary drivers and installing them can be done using Ubuntu's "Software & Updates" tool. This setting is found in the "Additional Drivers" tab of this tool. (Alternatively, the "Additional Drivers" tool may be available as a standalone application.)

When available drivers exist, a display such as:

* Using NVIDIA binary driver - version 375.29 from nvidia-375 (proprietary, tested)

will appear in the "Additional Drivers" area. Click this to select it and execute "Apply Changes" to install the driver and make it available. When multiple candidates are displayed, try to select ones with newer versions or those marked as "tested".

For Intel GPUs
^^^^^^^^^^^^^^

Intel CPU integrated GPUs such as HD Graphics and UHD Graphics use standard drivers that function in Ubuntu Linux. These are open source drivers, but Intel also seems to be involved in their development, and they appear to be comparable to Windows version drivers. Conversely, proprietary drivers for Linux do not exist for Intel GPUs.

.. _setup_gpu_ubuntu_gpu_driver_amd:

For AMD GPUs
^^^^^^^^^^^^

AMD GPUs seem to be able to use open source drivers by default in recent Ubuntu Linux.
AMD proprietary drivers may also be available in some cases, but since the situation regarding this matter changes rapidly, we will not go into detail here.
If the default drivers do not work properly or performance is poor, please obtain information from AMD's official website.

.. _setup_gpu_3d_rendering_engine:

Three-Dimensional Rendering Engine
-----------------------------------

Regarding the graphics environment, in addition to the GPU and its drivers, settings on the Choreonoid side also have an impact. This relates to the part called the "three-dimensional rendering engine" that handles the rendering of three-dimensional models in Choreonoid's implementation, for which the following two implementations are available:

1. New rendering engine (GLSL rendering engine, supports OpenGL 3.3 and later)
2. Old rendering engine (fixed shader rendering engine, supports OpenGL 1.5 and later)

In the latest development version, the new rendering engine is used by default. This uses OpenGL version 3.3 and later APIs and implements rendering functionality using GLSL, a programmable shader language.

The old rendering engine is inferior to the new engine in terms of rendering functionality and is no longer used by default. However, since it can be used with OpenGL version 1.5, it has a higher possibility of operating in environments where OpenGL is not sufficiently supported, such as PCs equipped with quite old GPUs or when used on virtual machines.

Normally, using the default new rendering engine is fine. If that does not work properly, you can switch to the old rendering engine.

.. note:: While Choreonoid previously used the old rendering engine as default, this was changed to use the new rendering engine as default in development versions from August 2019 onwards. Those who have been switching rendering engines previously should note this point.


How to Switch Rendering Engines
--------------------------------

Rendering engine switching is performed using the environment variable "CNOID_USE_GLSL".

If this variable is not defined or is set to the value 1, the new rendering engine is used. On the other hand, if this variable is set to 0, the old rendering engine will be used.


For Ubuntu Linux
^^^^^^^^^^^^^^^^^

When starting Choreonoid from the command line, using: ::

 CNOID_USE_GLSL=0 choreonoid ...

will cause the old rendering engine to be used. This method allows you to select the rendering engine each time you start. (Changing 0 to 1 will use the new rendering engine.)

If you set: ::

 export CNOID_USE_GLSL=0

in advance, the old rendering engine will be used without having to start as shown initially. In environments where the old rendering engine must be used, writing this in .profile in the home directory will eliminate the need to set it each time.

To return to the new rendering engine, use: ::

 unset CNOID_USE_GLSL

to clear the variable definition, or: ::

 export CNOID_USE_GLSL=1

For Windows
^^^^^^^^^^^

On Windows, you can also switch in the same way using the command prompt.
Of course, it is also possible to set this in the system environment variable settings.

How to Check OpenGL Version
----------------------------

On Windows, the OpenGL version can be checked using configuration tools provided by GPU manufacturers.

On Ubuntu, you can check using the "glxinfo" command.
This command can be installed by executing: ::

 sudo apt install mesa-utils

Then by executing: ::

 glxinfo

information about OpenGL available in that environment will be displayed. If there is a display such as: ::

 OpenGL version string: 4.5.0 NVIDIA 375.39

in this output, it means OpenGL up to version 4.5.0 is supported.

Alternatively, when Choreonoid starts, information such as: ::

 OpenGL 3.3 (GLSL 4.60) is available in the "Scene" view.
 Driver profile: ATI Technologies Inc. Radeon RX 5500 XT 3.3.14736 Core Profile Forward-Compatible Context 20.20.

is output to the :ref:`basics_mainwindow_messageview`, so you can also check there. (If the GLSL version is displayed at the end here, the new rendering engine is enabled. With the old rendering engine, the GLSL version is not displayed.)


How to Check GPU Performance
----------------------------

Pressing the "FPS Test" button in the :ref:`basics_sceneview_config_dialog` of the :ref:`basics_sceneview_sceneview` performs an animation that rotates the scene 360 degrees and displays the frame rate required for this. This functionality shows the rendering speed, so you can check changes in rendering speed when changing GPUs or GPU drivers. The test should be performed with some model or project loaded and the model being displayed.