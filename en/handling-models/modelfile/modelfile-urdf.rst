URDF Model Files
================

.. contents::
   :local:
   :depth: 3

Overview
--------

URDF (Unified Robot Description Format) is a robot model notation using XML. It is used by many platforms in recent years, including ROS (Robot Operating System). Note that the current official specification only supports the kinematic structure of robots, and sensors etc. are not supported.

Model file conventions
~~~~~~~~~~~~~~~~~~~~~~

This page is based on the `official URDF documentation <http://wiki.ros.org/urdf/XML/model>`_ and summarizes information necessary for robot simulation.

Points to note when creating URDF models are as follows:

* Describe one robot or environment model per model file.
* One robot must have a single tree structure for links. That is, it must be possible to reach any link from a certain link via joints, and the path to reach it must be unique. Therefore, closed mechanisms such as parallel link mechanisms cannot be expressed in URDF.
* All numerical values are in SI base units and derived units without prefixes. For example, meters or radians must be used for position units, and kilograms must be used for mass units.

Node and attribute list
-----------------------

In URDF, robot models are described by hierarchical node structures. Nodes with the same name have the same notation and role regardless of location.

Below is a list of nodes necessary for loading robot models in Choreonoid and performing simulations. It is based on the actual hierarchical structure of nodes. Not all URDF nodes are introduced, and explanations of nodes related to actual robots and operations described under joint nodes are omitted.

* :ref:`urdf-file-reference-robot-node` (attribute: `name`)

  * :ref:`urdf-file-reference-link-node` (attribute: `name`)

    * :ref:`urdf-file-reference-inertial-node`

      * origin node (attributes: `xyz`, `rpy`)
      * mass node (attribute: `value`)
      * inertia node (attributes: `ixx`, `iyy`, `izz`, `ixy`, `iyz`, `ixz`)

    * :ref:`urdf-file-reference-visual-node`

      * origin node (attributes: `xyz`, `rpy`)
      * geometry node

        * box node (attribute: `size`)
        * cylinder node (attributes: `radius`, `length`)
        * sphere node (attribute: `radius`)
        * mesh node (attributes: `filename`, `scale`)

      * material node (attribute: `name`)

        * color node (attribute: `rgba`)
        * texture node (attribute: `filename`)

    * :ref:`urdf-file-reference-collision-node`

      * origin node (attributes: `xyz`, `rpy`)
      * geometry node

        * box node (attribute: `size`)
        * cylinder node (attributes: `radius`, `length`)
        * sphere node (attribute: `radius`)
        * mesh node (attributes: `filename`, `scale`)

  * :ref:`body-file-reference-joint-node` (attributes: `name`, `type`)

    * origin node (attributes: `xyz`, `rpy`)
    * parent node (attribute: `link`)
    * child node (attribute: `link`)
    * axis node (attribute: `xyz`)
    * limit node (attributes: `lower`, `upper`, `velocity`, `effort`)

The role and notation of each node are detailed below.

.. _urdf-file-reference-robot-node:

robot node
----------

The robot node defines one robot. The robot definition must be completed entirely within the robot tag.

.. list-table:: Attributes of robot node
 :widths: 15,85
 :header-rows: 1

 * - Attribute name
   - Content
 * - name (required)
   - Robot name. Any string that does not duplicate within the model can be specified.

When loading in Choreonoid, a separate display name can be assigned, so even when loading multiple identical robots simultaneously, one model file is sufficient.

.. _urdf-file-reference-link-node:

link node
~~~~~~~~~

The link node defines each link that constitutes the robot.

.. list-table:: Attributes of link node
 :widths: 15,85
 :header-rows: 1

 * - Attribute name
   - Content
 * - name (required)
   - Link name. Any string that does not duplicate within the model can be specified.

The simplest link node is a node that only has a name. That is, ::

    <link name="simple_link"/>

A link can be configured by writing only this. Such links are often used for the purpose of marking robots, such as rotation centers of moving objects or reference points used for inverse kinematics of manipulators. However, this alone cannot perform physical simulation or rendering. Therefore, by describing the child nodes shown in the table below, additional information can be given to the link.

.. list-table:: Child nodes that link nodes can have
 :widths: 15,85
 :header-rows: 1

 * - Node name
   - Content
 * - :ref:`urdf-file-reference-inertial-node`
   - Defines center of mass position, mass, and moment of inertia.
 * - :ref:`urdf-file-reference-visual-node`
   - Defines appearance shape and color.
 * - :ref:`urdf-file-reference-collision-node`
   - Collision shape. Defines shape information used for detecting self-interference and collisions with the environment.

.. _urdf-file-reference-inertial-node:

inertial node
_____________

The inertial node defines the center of mass position, mass, and moment of inertia of the link used for physical simulation calculations. The description of the inertial node is optional, but if at least one link among fixedly connected links does not have an inertial tag set, physical simulation cannot be performed.

.. list-table:: Child nodes that inertial nodes can have
 :widths: 25,85
 :header-rows: 1

 * - Node name
   - Content
 * - origin node (optional)
   - Defines the center of mass position with the `xyz` attribute and the rotation of the moment of inertia coordinate system in roll-pitch-yaw representation with `rpy` on the link coordinate system. Each attribute can be omitted, and if omitted, it is set to zero. If the node itself is omitted, both `xyz` and `rpy` are treated as zero.
 * - mass node (required)
   - Defines mass with the `value` attribute. Mass must be a positive value.
 * - inertia node (required)
   - Defines moment of inertia. Specify the 6 components (`ixx`, `iyy`, `izz` `ixy`, `ixz`, `ixz`) that constitute the moment of inertia matrix as attributes. `ixx`, `iyy`, `izz` must always contain positive values. Also, since the moment of inertia must be a positive definite symmetric matrix, care must be taken when inputting `ixy`, `iyz`, `ixz` without using CAD or other information.

Example::

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>

.. _urdf-file-reference-visual-node:

visual node
___________

The visual node is used for rendering robot models. By describing the shape and color of links in this node, robot models can be displayed on various GUIs. Multiple visual nodes can be defined for one link, in which case the appearance of that link is the superposition of all visual nodes.

.. list-table:: Child nodes that visual nodes can have
 :widths: 25,85
 :header-rows: 1

 * - Node name
   - Content
 * - origin node (optional)
   - Defines the origin of the figure defined by the geometry node. The `xyz` attribute defines the center of mass position, and `rpy` defines the rotation of the coordinate system in roll-pitch-yaw representation, both on the link coordinate system. Each attribute can be omitted, and if omitted, it is set to zero. If the node itself is omitted, both `xyz` and `rpy` are treated as zero.
 * - geometry node (required)
   - Defines the shape of the figure. See the next table for details.
 * - material node (optional)
   - Specifies the color and texture of the figure specified by the geometry node. Details are described in the table below.

The geometry node is a node that defines geometric shapes. **Select one** from the nodes shown in the following table and describe it.

.. list-table:: List of child nodes that geometry nodes can have
 :widths: 15,85
 :header-rows: 1

 * - Node name
   - Content
 * - box node
   - Rectangular parallelepiped. The center becomes the origin. Give the length of sides parallel to each xyz axis in the `size` attribute.
 * - cylinder node
   - Cylinder. The center is the origin and the Z axis is the central axis of the circle. Give the radius with the `radius` attribute and the length with the `length` attribute. (Note: The central axis differs from Body files where the Y axis is the central axis of the circle.)
 * - sphere node
   - Sphere. The center becomes the origin. Give the radius with the `radius` attribute.
 * - mesh node
   - Mesh based on mesh files. Give the file path with the `filename` attribute. The path is described in the form `package://<packagename>/<path>` using the ROS package name and relative path within the package. Also, by giving the scaling factor for each axis in the `scale` attribute, the mesh can be scaled. If color information is included in the mesh file, that information will be used unless overridden by the material node below.

The material node specifies surface color and texture for geometric shapes. If no material node is specified, white is applied in principle.

.. list-table:: List of child nodes that material nodes can have
 :widths: 15,85
 :header-rows: 1

 * - Node name
   - Content
 * - color node
   - Figure color. By giving four parameters to the `rgba` attribute, RGB and transparency are specified. Each value uses normalized values between 0.0 and 1.0.
 * - texture node
   - Texture on the figure surface. Give the file path with the `filename` attribute. The path is described in the form `package://<packagename>/<path>` using the ROS package name and relative path within the package, similar to the mesh node in the geometry node.

Also, the material node can be given a name with the name attribute. Once defined, a material node can be applied with the same appearance decoration by simply specifying the name like `<material name="material1"/>` within the subsequent model file.

Example 1::

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1 1 1"/>
      </material>
    </visual>

Example 2::

    <visual>
      <geometry>
        <mesh filename="package:://choreonoid/share/model/JACO2/parts/ARM.stl" scale="1 1 1"/>
      </geometry>
    </visual>

.. _urdf-file-reference-collision-node:

collision node
______________

The collision node is used for collision detection of robot models. The information from collision nodes is used when detecting self-interference and contact with the environment in physical simulation and planning. Like visual nodes, multiple collision nodes can be defined for one link, and the collision model for that link is the superposition of all collision nodes.

.. list-table:: Child nodes that collision nodes can have
 :widths: 25,85
 :header-rows: 1

 * - Node name
   - Content
 * - origin node (optional)
   - Defines the origin of the figure defined by the geometry node. The `xyz` attribute defines the center of mass position, and `rpy` defines the rotation of the coordinate system in roll-pitch-yaw representation, both on the link coordinate system. Each attribute can be omitted, and if omitted, it is set to zero. If the node itself is omitted, both `xyz` and `rpy` are treated as zero.
 * - geometry node (required)
   - Defines the shape of the figure. Please refer to the explanation of the geometry node in :ref:`urdf-file-reference-visual-node` for details. However, when using mesh nodes, even if color information is included in the mesh file, that information becomes invalid in collision nodes.

Example 1::

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>

Example 2::

    <collision>
      <geometry>
        <mesh filename="package:://choreonoid/share/model/JACO2/parts/ARM.stl" scale="1 1 1"/>
      </geometry>
    </collision>

.. _body-file-reference-joint-node:

joint node
~~~~~~~~~~

The joint node defines the relationship between two links.

.. list-table:: Attributes of joint node
 :widths: 15,85
 :header-rows: 1

 * - Attribute name
   - Content
 * - name (required)
   - Joint name. Any string that does not duplicate within the model can be specified.
 * - type (required)
   - Joint type. Specify one of the following: revolute joint with range of motion: `revolute`, joint that rotates infinitely like wheels: `continuous`, linear joint: `prismatic`, fixed relationship `fixed`, or completely unconstrained `floating`. (There is also `planar` that moves on a plane, but since Choreonoid does not support it, the explanation is omitted.)

To further define joints, it is necessary to specify parent-child relationships between links, relative positions, range of motion, etc. Therefore, this information is described in the child nodes shown in the table below.

.. list-table:: Child nodes that joint nodes can have
 :widths: 30,85
 :header-rows: 1

 * - Node name
   - Content
 * - origin node (optional)
   - Defines the relative position and orientation between two links. The `xyz` attribute gives the relative position of the child link origin when joint displacement is zero, as seen from the parent link origin. The `rpy` gives the relative orientation of the child link coordinate system as seen from the parent link coordinate system in roll-pitch-yaw representation. Each attribute can be omitted, and if omitted, it is set to zero. If the node itself is omitted, both `xyz` and `rpy` are zero, meaning the origin positions and orientations of the parent and child links coincide.
 * - parent node (required)
   - Defines the parent link by specifying the link name in the `link` attribute.
 * - child node (required)
   - Defines the child link by specifying the link name in the `link` attribute.
 * - axis node (optional)
   - Specifies the joint axis. Specify a 3D vector representing the direction of the joint axis as seen from the parent link coordinate system in the `xyz` attribute. This is valid only when the `type` attribute of the joint node is `revolute`, `continuous`, or `prismatic`. For revolute joints, it specifies the rotation axis, and for linear joints, it specifies the direction of motion. Note that the direction determines the sign of joint displacement. If there is no node, the default value is `xyz="1 0 0"`, that is, the x-axis direction in the parent link coordinate system.
 * - limit node (conditionally required)
   - Defines the range of motion, velocity, and actuator output of the joint. Required when the `type` attribute of the joint node is `revolute` or `prismatic`. The `lower` attribute specifies the negative motion limit, and the `upper` attribute specifies the positive motion limit. Both attributes representing the range of motion have a default value of 0, and if both are not specified, the joint becomes immobile. Additionally, the `velocity` attribute gives the upper limit of joint velocity (units are [m/s] or [rad/s]), and the `effort` attribute gives the upper limit of actuator output (units are [N] or [Nm]). Both velocity upper limit and output upper limit must be positive values, and there are no default values for these.

Example::

    <joint name="sample_joint" type="revolute">
      <origin xyz="0 0 1" rpy="0 0 3.1416"/>
      <parent link="link1"/>
      <child link="link2"/>
      <axis xyz="1 0 0"/>
      <limit lower="-3.14" upper="3.14" velocity="1.0" effort="30"/>
    </joint>

.. _urdf-file-reference-sample-model:

Sample model
------------

Choreonoid provides a URDF version of the `SR1 <modelfile-sr1.html>`_, which is a sample model of a humanoid robot. You can understand the correspondence by comparing it with the Body file version.

* Body: https://github.com/choreonoid/choreonoid/blob/master/share/model/SR1/SR1.body
* URDF: https://github.com/choreonoid/choreonoid/blob/master/share/model/SR1/SR1.urdf