
Optional Functions Overview
===========================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

This section describes the main optional functions that can be selected for the build configuration of the development version of Choreonoid.

* **BUILD_ASSIMP_PLUGIN**

 Builds an Assimp plugin that uses the Assimp library and enables you to read 3D model files in formats such as COLLADA. It is ON by default.
 
* **ENABLE_PYTHON**

 Build any Python wrapper available for each module and plug-in. To use it, it is necessary to install the library for development of Python.

 For a module, the library function of Choreonoid can be used from Python by using Python wrapper in combination. For example, by importing "cnoid Body" module, which is Python wrapper of Body wrapper, it is also possible to load the robot model file on Python and make different calculations. As for plug-in Python wrapper, it will become possible to use the plug-in function on Choreonoid from Python script by using it in combination with Python plug-in described below.

* **BUILD_PYTHON_PLUGIN**

 It builds Python plug-in. It is necessary to turn ON ENABLER_PYTHON, too. This plug-in makes possible reading and running Python script and the functions that operates on Choreonoid including Python console.

* **BUILD_PYTHON_SIM_SCRIPT_PLUGIN**

 It builds PythonSimScript plug-in to use Python script for the simulation setup. It is also necessary to build Python plug-in.

* **BUILD_POSE_SEQ_PLUGIN**

 It builds PoseSeq plug-in, which provides provides the choreographic function by key pause. It is configured to ON by default.

* **BUILD_BALANCER_PLUGIN**

 It builds Balancer plug-in, which provides the auto-balance correction for the choreographic function. When choreographing a biped robot, you can make it not fall down (logically) by using this function.

* **BUILD_SIMPLE_CONTROLLER_SAMPLES**

 It builds sample projects using controllers implemented as a SimpleController

* **BUILD_ODE_PLUGIN**

 It builds ODE plug-in. With this plug-in, "Open Dynamics Engine (ODE)", which is a library of open source dynamics calculations, can be used as a calculation engine for the simulation function of Choreonoid. To use this, it is necessary to install `Open Dynamics Engine (ODE) <http://www.ode.org/>`_ .

* **BUILD_BULLET_PLUGIN**

 Builds the Bullet plugin. This plugin lets you make use of the Bullet Physics Library, a dynamics computation library, as a computation engine for simulations in Choreonoid. For more details about Bullet, refer to the `Bullet Physics Library <http://bulletphysics.org>`_ . For the build process, refer to the section on  :doc:`build-bulletPlugin` 


* **BUILD_PhysX_PLUGIN**

 Builds the PhysX plugin. This plugin lets you make use of PhysX, a dynamics computation library, as a computation engine for simulations in Choreonoid. For more details about PhysX, refer to the `PhysX website <https://developer.nvidia.com/physx-sdk>`_ . For the build process, refer to the section on  :doc:`build-physxPlugin`  .
  
* **ENABLE_CORBA**

 It builds the modules that are the bases of CORBA-related functions. To introduce these modules, it is necessary to install `omniORB <http://omniorb.sourceforge.net/>`_ library.

* **BUILD_CORBA_PLUGIN**

 This plug-in provides CORBA-related functions. EMABLE_CORBA must be enabled, too.

* **BUILD_MEDIA_PLUGIN**

 It builds Media plug-in that replays a media file. Some platforms require different libraries to handle media files.
