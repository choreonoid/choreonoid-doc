Robot Models
============

In :doc:`overview`, we introduced :ref:`wrs2018_overview_robots`. This section explains the model data for handling each robot in Choreonoid.

.. contents::
   :local:

Model Files
-----------

Choreonoid models are typically described as "Body format" files with the "body" extension. The specifications of this format are detailed in the :doc:`../handling-models/modelfile/index` section of this manual, so please refer to that for details.

When adjusting parameters or modifying standard robot models, or creating new robot models, you need to edit model files in this format. Model files are text data in YAML format, so they are typically edited with a text editor. For information on how to edit them, please see :doc:`../handling-models/modelfile/modelfile-newformat`.

Model files can be loaded into Choreonoid as "body items." For information on handling body items, please see :doc:`../handling-models/index`.

.. _wrs_standard_model_directory:

Standard Model Directory
------------------------

Choreonoid includes numerous model files as samples. These are stored under Choreonoid's "standard model directory." In the source code, the standard model directory is "share/model". If you install Choreonoid using make install, it becomes "share/choreonoid-x.x/model" in the installation destination. (x.x is the version number. See :doc:`../install/directories`)

The standard robot models for WRS2018 are also included as Choreonoid sample models and are stored under the standard model directory.

WAREC-1
-------

The WAREC-1 model is stored in a directory called "WAREC1" under the standard model directory. The main file is "WAREC1.body", and by loading this file from Choreonoid, you can perform simulations of WAREC1.

Double-Arm Construction Robot
-----------------------------

The double-arm construction robot model is stored in a directory called "DoubleArmV7" under the standard model directory.

This model uses crawlers, and we provide both a version that performs simplified crawler simulation and a version that performs more realistic simulation using AGX Dynamics. The base model name "DoubleArmV7" is suffixed with "S" for "Simplified" or "A" for "AGX":

* Simplified crawler version: DoubleArmV7S.body
* AGX crawler version: DoubleArmV7A.body

The simplified crawler version can be used with Choreonoid's standard features, and WRS2018 samples are also available. This allows those without an AGX Dynamics license to try WRS2018 simulations. However, please note that the crawler behavior differs from the actual machine.

The AGX crawler version provides crawler behavior closer to the actual machine. If you have an AGX Dynamics license, please use this version. This version is also used in the competition.

Aizu Spider
-----------

The Aizu Spider model is stored in a directory called "AizuSpider" under the standard model directory.

This model comes in six variations. First, regarding the arms mounted on the robot, we provide three variations: no arms (N), single arm (S), and dual arms (D). Additionally, this model is equipped with crawlers, which come in simplified (S) and AGX (A) versions. These combinations result in the following six models:

* No arms, simplified crawler version: AizuSpiderNS.body
* No arms, AGX crawler version: AizuSpiderNA.body
* Single arm, simplified crawler version: AizuSpiderSS.body
* Single arm, AGX crawler version: AizuSpiderSA.body
* Dual arms, simplified crawler version: AizuSpiderDS.body
* Dual arms, AGX crawler version: AizuSpiderDA.body

Note that the arms mounted on this robot are commercial JACO2 arms from Kinova.

Quadcopter
----------

The Quadcopter model is stored in the "multicopter" directory under the standard model directory with the filename "quadcopter.body". To perform flight simulations of this model, the :doc:`../multicopter/index` is required.

.. _wrs2018_model_creation_note:

Notes on Model Creation/Modification
------------------------------------

When adjusting parameters or modifying standard robot models, or creating new robot models, please note the following:

* When using AGX crawlers, appropriate material settings for the crawlers must be configured. For details, please see :ref:`agx_continous_track_material`. There's no problem if you use the sample models as-is, but if you change model or link names or create new crawlers, you need to prepare corresponding :ref:`agx_material_file`. AGX crawlers without appropriate material settings will not operate as expected.

* When mounting/adding sensors such as cameras and range sensors, or devices such as lights, be careful about their quantity. Mounting numerous sensors and devices will slow down simulation and rendering accordingly. You need to keep the scale to a level that allows real-time simulation in the PC environment with the specifications expected at the competition.