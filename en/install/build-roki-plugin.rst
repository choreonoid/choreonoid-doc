Building the RoKi Plugin
===================

.. contents:: Table of Contents
   :local:

.. highlight:: YAML

Installing RoKi
---------------

To use the RoKi Plugin, you need to install RoKi (Robot Kinetics library) beforehand. RoKi is only supported on Linux.

First, install CURE.

Navigate to the CURE page from the `RoKi homepage <http://www.mi.ams.eng.osaka-u.ac.jp/open-j.html>`_, download cure-1.0.0-beta5.tgz and extract it.
 
There is a file called README, so basically follow the instructions in it for installation.

.. note:: The default installation destination is **home/usr**. The following explanation is for installing to this location. If you want to change the installation destination, edit the config file and modify the following content accordingly.
 
It seems better to create the installation destination folder in advance, and also create bin and lib folders within it. Start a terminal and in the home directory, do the following: ::

 mkdir usr
 cd usr
 mkdir bin
 mkdir lib

Add this directory to the environment variables PATH and LD_LIBRARY_PATH. Open the .profile file in the home directory with an editor. Paste the following content at the end of the file and save it. ::

 export PATH=$PATH:$HOME/usr/bin
 export LD_LIBRARY_PATH=$HOME/usr/lib:$LD_LIBRARY_PATH

To apply this setting, execute: ::

 source .profile
 
Move to the directory where you extracted the Cure source and execute the following: ::

 make
 make install
 
If successful, a file called cure-config should be created under home/usr/bin, and libcure.so should be created under home/usr/lib.

Next, navigate to the ZM page, download the file, and extract it. Move to the extracted directory and perform make, make install.

Install Zeo and Roki in the same manner in that order.

Building the Plugin
-------------------

When building choreonoid, set the **BUILD_ROKI_PLUGIN** item to "ON" in the CMake configuration, and specify the RoKi installation destination in **ROKI_DIR**.

Running Simulations
-------------------

Simulations using the RoKi Plugin are performed in the same way as :ref:`using other physics simulators<simulation_creation_and_configuration_of_simulator_item>`. You can execute simulations by creating a simulator item "RoKiSimulator" and placing it as a child item of the world item.

Properties of RoKiSimulator Item
--------------------------------

The RoKiSimulator item has the following properties.

.. list-table:: RoKi Plugin Properties
 :widths: 15,60
 :header-rows: 1

 * - Property
   - Description
 * - staticfriction
   - Static friction
 * - kineticfriction
   - Kinetic friction
 * - contactType
   - Type when objects contact: select rigid or elastic
 * - solverType
   - Select Vert or Volume
 * - compensation
   - Contact parameter, effective when contactType is rigid and useContactFile is false
 * - relaxation
   - Contact parameter, effective when contactType is rigid and useContactFile is false
 * - elasticity
   - Contact parameter, effective when contactType is elastic and useContactFile is false
 * - viscosity
   - Contact parameter, effective when contactType is elastic and useContactFile is false
 * - useContactFile
   - Whether to load contact parameter settings from a file
 * - contactFileName
   - Contact parameter setting file name, effective when useContactFile is true

For details about the parameters, please refer to the `RoKi homepage <http://www.mi.ams.eng.osaka-u.ac.jp/open-j.html>`_.

Joint Dynamics Simulation
-------------------------

RoKi enables joint dynamics simulation. The sample project for this is RokiArm2Dof.cnoid.

The joint dynamics parameters are described in the model file arm_2dof.body. Since this model applies the same parameters to two joints, it uses **import** in the :ref:`body-file-reference-link-node`. For information about the alias feature, see :ref:`modelfile_yaml_alias`. When setting different parameters for each joint, write them directly in the Link node. ::

 actuator1: &actuator1
   rotorInertia: 1.65e-6
   gearRatio: 120.0
   gearInertia: 5.38e-6
   motorAdmittance: 0.42373
   motorConstant: 2.58e-2
   motorMinVoltage: -24.0
   motorMaxVoltage: 24.0
   jointStiffness: 0.0
   jointViscosity: 2.2
   jointFriction: 4.32
   jointStaticFriction: 4.92
  
 links:
    .......
   -
     name: Joint1
      .......
     import: *actuator1
      .......
   -
     name: Joint2
      .......
     import: *actuator1
      .......
      
The joint parameters are as follows.

.. list-table:: 
 :widths: 15,40
 :header-rows: 1

 * - Parameter
   - Description
 * - motorconstant
   - Motor constant (torque constant)
 * - admitance
   - Terminal admittance (reciprocal of terminal resistance)
 * - minvoltage
   - Minimum input voltage
 * - maxvoltage
   - Maximum input voltage
 * - inertia
   - Motor rotor moment of inertia
 * - gearinertia
   - Gear reducer moment of inertia
 * - ratio
   - Reduction ratio
 * - stiff
   - Joint stiffness coefficient
 * - viscos
   - Joint viscosity coefficient
 * - coulomb
   - Joint dry coefficient (kinetic friction torque)
 * - staticfriction
   - Maximum static friction torque

Fracture Simulation
------------------

RoKi enables fracture simulation by describing the locations where fractures occur as joints in the model file. The sample project for this is RokiBreakWall.cnoid.

The fracturing model is described in breakWall.body. Define the locations where fractures occur as joints, and set the joint type to free. Then, describe the force and torque norm thresholds at which fractures occur in the **break** parameter, in the order of force and torque. ::

 links :
   -
    name: BASE
    jointType: fixed
     ................
    elements:
      Shape:
        geometry: { type: Box, size: [ 0.099, 0.049, 0.099 ] }
   -
    name: link1
    parent: BASE
    translation : [ 0, 0, 0.05 ]
    jointType: free
      .............
    break: [ 200.0, 200.0 ]
      .............
    elements:
      Shape:
        geometry: { type: Box, size: [ 0.099, 0.049, 0.099 ] }
  -
    name: link2
    parent: link1
    translation : [ 0, 0, 0.1 ]
    jointType: free
      .............
    break: [ 10.0, 10.0 ]
      .............
      
To prevent the fractured objects from passing through each other, the self-collision detection property of the breakWall model item must be set to true. However, this would cause self-collision even before fracturing. To avoid this, the breakWall model sets the geometric shapes of the links so that there are slight gaps between the links.

Set the all link position and orientation output property of the RoKiSimulation item to true.