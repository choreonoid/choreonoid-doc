Body File Tutorial
==================

This page provides a tutorial on how to write "Body files," which are Choreonoid's standard model file format.

.. contents::
   :local:
   :depth: 2

.. highlight:: YAML

.. _bodyfile-tutorial-simple-tank-model:

SimpleTank Model
----------------

The model we will be working with is the "SimpleTank" model shown below.

.. image:: images/tank.png

This is a model composed of two rotational joints that move the turret and gun barrel, and two crawlers for movement, with a camera and light mounted as devices.

SimpleTank is a simplified version of the "Tank" model, which is a standard sample of a crawler-type mobile robot, and has the same basic structure as the Tank model. Sample projects such as:

* TankJoystick.cnoid
* TankVisionSensors.cnoid

are included in Choreonoid itself for the Tank model.

This manual also includes :doc:`../../simulation/tank-tutorial/index`, which explains how to perform simulations using this model.

Basic Model Structure
---------------------

The SimpleTank model consists of five parts as shown in the figure below.

.. image:: images/tank_decomposed.png

The base part is the chassis. The upper part of the chassis is equipped with a turret and gun barrel. This part consists of two components: a base for the turret that performs yaw axis rotation, and a part mounted on top of it that performs pitch axis rotation along with the gun barrel. Crawler mechanisms for movement are attached to the left and right sides of the chassis.

These five parts are modeled as "links." The chassis part is the central part of the model and is modeled as the "root link." Each model must have exactly one root link defined. The two turret links are each modeled as rotational joints. The crawler parts are modeled as links corresponding to :doc:`../../simulation/pseudo-continuous-track`.

The hierarchical structure (parent-child relationships) between these links is as follows: ::

 - Chassis (root)
     + Turret yaw axis part (rotational joint)
            + Turret pitch axis part (rotational joint)
     + Left crawler
     + Right crawler

In this tutorial, we will describe the shape of each link as text directly in the model file. This allows us to complete the modeling with just text files, without using shape data created with CAD or modeling tools. However, it is also possible to use shape data created with CAD or modeling tools. Please refer to :doc:`tank-blender` for more information on this.

Preparing the Model File
------------------------

Body format model files are created as text files. The file extension is usually ".body".

To start creating a model file, first create an empty text file using a text editor and save it with an appropriate filename with the ".body" extension. In this case, we'll save it as "simpletank.body". The completed file is stored in the "model/tank" directory of Choreonoid's share directory. This tutorial will explain the contents of that file while showing an example of the creation process leading to completion.

The complete description can be found in :doc:`tank`.

.. note:: When creating model files using Ubuntu's standard text editor "gedit," selecting "YAML" in the settings dialog displayed from the main menu's "View" - "Highlight Mode" will provide syntax highlighting suitable for YAML format, making it easier to edit.

About YAML
----------

Body files use YAML as the base description format. While you can generally understand how to write YAML by reading the explanation below, for more detailed information, please refer to the `YAML specification <http://www.yaml.org/spec/1.2/spec.html>`_ and various tutorial articles.

Writing the Header
------------------

First, write the following as the model file header using YAML mapping: ::

 format: ChoreonoidBody
 format_version: 2.0
 name: SimpleTank

The first line allows this file to be recognized as a Choreonoid model file.

format_version is used to distinguish between different versions of the description format.
Currently, version 2.0 is the latest, so specify 2.0 unless there is a specific reason not to.

The model name is written in "name". Here we set it as "SimpleTank".

.. note:: Keys used in Body files are basically named in "snake case" format. This means writing all key strings in lowercase and connecting multiple words with underscores (_). The "format_version" above is also in this format. However, in older versions of Choreonoid, all keys were named in "lower camel case." This capitalizes the first letter of each word boundary, resulting in descriptions like "formatVersion". Previously defined lower camel case keys can still be read for compatibility, but please use the new format going forward. Note that some keys are still only defined in lower camel case.

.. note:: When the description format version is 1.0, you can also specify the unit for angles described in the Body file. Specifically, if you specify "degree" for "angleUnit", it uses degrees, and if you specify "radian", it uses radians. However, this can cause confusion, so version 2.0 always uses degrees for description.

.. _modelfile_yaml_links:

Writing Links
-------------

Information about the links that the model has is written in "links:" as follows: ::

 links:
   -
     Description of link 1 (root link)
   -
     Description of link 2
   -
     Description of link 3
   ...

This way, you can describe any number of links as a YAML list. The description part of each link is called a "Link node." The first Link node described is considered the model's root link.

.. _modelfile_yaml_link_node:

Link Node
---------

Link nodes are written in YAML mapping format. The following parameters are available as mapping elements.

.. list-table::
 :widths: 20, 80
 :header-rows: 1

 * - Key
   - Content
 * - name
   - Link name
 * - parent
   - Parent link. Specified by the parent link's name (string written in name). Not used for root link
 * - translation
   - Relative position of this link's local frame from parent link. For root link, used as default position when loading model
 * - rotation
   - Relative orientation of this link's local frame from parent link. Orientation is expressed with 4 numbers corresponding to rotation axis and rotation angle (Axis-Angle format). For root link, used as default orientation when loading model
 * - joint_type
   - Joint type. Specify one of **fixed** (fixed), **free** (unfixed root link), **revolute** (rotational joint), **prismatic** (linear joint), **pseudo_continuous_track** (simple continuous track)
 * - joint_axis
   - Joint axis. Specify the direction of the joint axis as a list of 3 elements of a 3D vector. The value should be a unit vector. If the joint axis matches any of X, Y, Z in the link's local coordinates, it can also be specified by the corresponding axis character (one of X, Y, Z).
 * - joint_range
   - Joint range of motion. List the minimum and maximum values as two values. By writing the value as unlimited, you can remove range restrictions. If the absolute values of minimum and maximum are the same with negative and positive signs respectively, you can write just the absolute value (as a scalar value)
 * - joint_id
   - Joint ID value. Specify an integer value of 0 or greater. Any value that doesn't duplicate within the model can be specified. If the link is not a joint (root link or joint_type is fixed) or if access by ID value is not needed, it doesn't need to be specified
 * - center_of_mass
   - Center of mass position. Specified in link local coordinates
 * - mass
   - Mass [kg]
 * - inertia
   - Moment of inertia. List 9 elements of the inertia tensor matrix. Due to the symmetry of the inertia tensor, you may list only the 6 elements of the upper triangular part.
 * - elements
   - Describe child nodes that constitute the link's components


Writing the Chassis Link
------------------------

Let's first write the root link corresponding to the chassis part of this model. Write the corresponding Link node under links as follows: ::

 links:
   -
     name: CHASSIS
     translation: [ 0, 0, 0.1 ]
     joint_type: free
     center_of_mass: [ 0, 0, 0 ]
     mass: 8.0
     inertia: [
       0.1, 0,   0,
       0,   0.1, 0,
       0,   0,   0.5 ]
     elements:
       Shape:
         geometry:
           type: Box
           size: [ 0.45, 0.3, 0.1 ]
         appearance: &BodyAppearance
           material:
             diffuse: [ 0, 0.6, 0 ]
             specular: [ 0.2, 0.8, 0.2 ]
             specular_exponent: 80

Since indentation on each line also defines the data structure in YAML, be careful to maintain proper indentation alignment in the above description.

In the link definition, first set a name to identify the link. Here, we have: ::

 name: CHASSIS

which sets the name to "CHASSIS".

Checking the Model Being Edited
-------------------------------

Although we have only written the root link, this already constitutes a valid model. Let's load the file being edited in Choreonoid to display it and check if it's written correctly. Select "File" - "Load" - "Body" from the main menu and select the target file in the dialog that appears. If you enable "Check in item tree view" on the dialog or click the item's checkbox after loading, it should be displayed in the scene view as follows.

.. image:: images/tank_chassis.png

If an error occurs when loading the item or if it doesn't display properly after loading, please check the content written so far.

When reloading a model file after modification, if the previous file is already loaded as a body item, you can easily reload it using the item's "reload function". This can be done by either of the following operations:

* Select the target item in the item tree view and press **"Ctrl + R"**.
* Right-click the target item in the item tree view and select "Reload" from the menu that appears.

When you reload, the updated file is reloaded immediately, and (if there are no loading errors) the current item is replaced with it. If there are changes to the shape or other aspects in the updated file, the display in the scene view immediately reflects this. Using this function, you can edit model files relatively efficiently while directly editing text files. You will perform this "reload" operation many times as you progress through this tutorial, so please remember it.


Root Link-Specific Description
------------------------------

In the CHASSIS link, we have: ::

 translation: [ 0, 0, 0.1 ]

which sets the initial position when loading the model. (To be precise, this is the position of the root link origin in the world coordinate system.)

translation is normally a parameter that represents the relative position from the parent link, but the root link has no parent link. Instead, it is considered as the relative position from the world coordinate origin when loading the model. Note that the initial orientation can also be set using rotation. If you don't care about the initial position, you don't need to set these parameters.

Here, by setting the Z coordinate value to 0.1, we set the initial position of the root link to be raised 0.1[m] in the Z-axis direction. This allows the bottom surface of the crawlers to coincide exactly with the Z=0 plane when loaded, while keeping the root link origin at the center of the chassis. Since environmental models often use this as the floor surface, the above setting makes it easy to align with that.

Next, with: ::

 joint_type: free

we set that this model is a model that can move freely in space.

joint_type is normally a parameter that specifies the type of joint connecting parent and child links, but for the root link the meaning is slightly different - it specifies whether the link is fixed to the environment or not. If you specify "fixed" here, the link becomes fixed, so set it that way for manipulators whose base is fixed to the floor. On the other hand, for models like this one that are not fixed to a specific location, specify "free" here.


.. _modelfile_yaml_rigidbody_parameters:

Writing Rigid Body Parameters
-----------------------------

Each link is usually modeled as a rigid body. The :ref:`modelfile_yaml_link_node` for describing this information includes center_of_mass, mass, and inertia. For the CHASSIS link, these are written as follows:

.. code-block:: yaml
 :dedent: 0

     center_of_mass: [ 0, 0, 0 ]
     mass: 8.0
     inertia: [
       0.1, 0,   0,
       0,   0.1, 0,
       0,   0,   0.5 ]

center_of_mass describes the center of mass position in the link's local coordinates. The local coordinate origin of the CHASSIS link is set at the center of the chassis, and the center of mass is also set to coincide with it.

mass specifies the mass, and inertia specifies the matrix elements of the inertia tensor.

Here we have set appropriate values for the inertia tensor.
In practice, please set appropriate values using proper calculations or CAD tools.

Since the inertia tensor is a symmetric matrix, it's OK to write only the 6 elements of the upper triangular part. In this case, the above values can be written as:

.. code-block:: yaml
 :dedent: 0

     inertia: [
       0.1, 0,   0,
            0.1, 0,
                 0.5 ]

Note that rigid body parameters can also be written independently using a "RigidBody" node. This will be explained later.


.. _modelfile_yaml_chassis_shape:

Writing the Chassis Shape
-------------------------

The shape of the link is written under "elements" in the Link node. For the CHASSIS link, it is written as follows:

.. code-block:: yaml
 :dedent: 0

       Shape:
         geometry:
           type: Box
           size: [ 0.45, 0.3, 0.1 ]
         appearance: &BodyAppearance
           material:
             diffuse: [ 0, 0.6, 0 ]
             specular: [ 0.2, 0.8, 0.2 ]
             specular_exponent: 80

This part is a "Shape node". The shape displayed in the scene view when you loaded the model file earlier is described here.

In the Shape node, geometry describes the geometric shape and appearance describes the surface appearance.

This time we specified "Box" for the geometry type to describe a Box node that represents a box-shaped (cuboid) geometric shape. In the Box node, the lengths in the x, y, and z axis directions are written as a list for the size parameter. Other shape nodes such as sphere (Sphere), cylinder (Cylinder), and cone (Cone) can also be used.

For appearance, we describe material which describes the surface material. The following parameters can be set in material:

.. list-table::
 :widths: 20, 80
 :header-rows: 1

 * - Key
   - Content
 * - ambient
   - Specifies the scalar value of the reflection coefficient for ambient light. The value range is 0.0 to 1.0. Default is 0.2.
 * - diffuse
   - Describes the RGB values of the diffuse reflection coefficient. RGB values are a list of three components for red, green, and blue, with each component value ranging from 0.0 to 1.0.
 * - emissive
   - Specifies the RGB values of the emissive color. Default is disabled (all components are 0).
 * - specular
   - Describes the RGB values of the specular reflection coefficient. Default is disabled (all components are 0).
 * - specular_exponent
   - A parameter that controls the sharpness of specular reflection. Larger values make highlights smaller and sharper, creating an appearance like metal or polished surfaces. Set a value of 0 or greater. Default is 25. Values around 100 start to look metallic.
 * - shininess
   - An old parameter that controls the sharpness of specular reflection. This is specified in the range 0 to 1. Do not use this parameter in the future.
 * - transparency
   - Specifies transparency. The value is a scalar from 0.0 to 1.0, where 0.0 is completely opaque and 1.0 is completely transparent. Default is 0.0.

Here we set the three parameters diffuse, specular, and specular_exponent to represent a green material with somewhat metallic luster.

.. note:: For such shape descriptions, although the syntax and key names differ somewhat, the structure, shape types, and parameters largely follow those defined in `VRML97 <http://tecfa.unige.ch/guides/vrml/vrml97/spec/>`_ (such as `Shape <http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Shape>`_, `Box <http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Box>`_, `Sphere <http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Sphere>`_, `Cylinder <http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Cylinder>`_, `Cone <http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Cone>`_, `Appearance <http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Appearance>`_, `Material <http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Material>`_, etc.). Since VRML97 was the format used as the base for OpenHRP format model files, those with experience using it should find it easy to understand.

.. note:: As mentioned at the beginning, in this tutorial we describe the shape of each link as text directly in the model file using the above description method. It is also possible to use shape data files created separately with modeling tools or CAD tools. This is explained in other documents.

.. _modelfile_yaml_anchor:

Setting Anchors
---------------

In the above code, we have: ::

 appearance: &BodyAppearance

where "&BodyAppearance" is added immediately after appearance.

This corresponds to YAML's "anchor" feature, which allows you to name a specific part of YAML and reference it later. This makes it possible to omit repetitive descriptions by setting an anchor on the first description and referencing it for the rest. The part that references an anchor is called an "alias" in YAML.

Since we will apply the same material parameters set for appearance in :ref:`modelfile_yaml_turret_pitch_shape`, we set an anchor here so it can be reused there. The actual usage is described in :ref:`modelfile_yaml_alias`.

.. _modelfile_yaml_elements:

Writing elements
----------------

In model files, a collection of information about a component is called a "node". We have introduced Link nodes and Shape nodes as examples so far.

Some nodes can contain subordinate nodes as their child nodes. This allows nodes to be described hierarchically. A general method for doing this is the elements key.

In elements, child nodes are basically written using YAML's list notation as follows: ::

 elements:
   -
     type: Node type name
     key1: value1
     key2: value2
     ...
   - 
     type: Node type name
     key1: value1
     key2: value2
   ...


If subordinate nodes can also contain elements, you can deepen the hierarchy as follows: ::

 elements:
   -
     type: Node type name
     key1: value1
     elements:
       -
         type: Node type name
         key1: value1
         elements:
           ...

In this way, using elements makes it possible to describe structures that combine multiple types of nodes.

Note that if only one node of a certain type is included under elements, the following simplified notation can also be used: ::

 elements:
   Node type name:
      key1: value1
      key2: value2
      ...

There's not much difference from the previous one, but this notation is slightly simpler as it doesn't use list notation.

Link nodes can contain various elements such as shapes and sensors using this elements. Other nodes that can use elements include Transform and RigidBody nodes.

.. note:: When a model has multiple links, the relationships between links are generally hierarchical. While it might be considered to describe this using elements of Link nodes, this format of model files does not use such description. This is because such description would deepen the text hierarchy in the model file as the link hierarchy deepens, making it difficult to check and edit as text. The link hierarchy is described using the "parent" key of Link nodes.

Writing the Turret Yaw Axis Link
--------------------------------

Next, let's write the link for the yaw axis part that serves as the base of the turret. Add the following to the previous description.

.. code-block:: yaml
 :dedent: 0

   -
     name: TURRET_Y
     parent: CHASSIS
     translation: [ -0.04, 0, 0.1 ]
     joint_type: revolute
     joint_axis: -Z
     joint_range: unlimited
     max_joint_velocity: 90
     joint_id: 0
     center_of_mass: [ 0, 0, 0.025 ]
     mass: 4.0
     inertia: [
       0.1, 0,   0,
       0,   0.1, 0,
       0,   0,   0.1 ]
     elements:
       Shape:
         geometry:
           type: Box
           size: [ 0.2, 0.2, 0.1 ]
         appearance: *BodyAppearance

After writing this, save the file and perform the reload mentioned earlier. The model display in the scene view should look like this:

.. image:: images/tank_turret_y.png

The newly added part on top of the chassis is the turret base. This part is designed to rotate around the yaw axis and includes the joint for that purpose.

As specified in name, this link is named "TURRET_Y". This indicates that it is the Yaw axis of the turret. Also, like the CHASSIS link, we have written the rigid body parameters center_of_mass, mass, and inertia.

For the shape, we are using a Box type geometry like the CHASSIS link. By adjusting its size parameter, we have made it an appropriately sized shape for the turret base.

.. _modelfile_yaml_alias:

Using Aliases
-------------

In the shape description above, since the appearance can be the same as the CHASSIS link, we will reuse the content set in :ref:`modelfile_yaml_chassis_shape`. We set an :ref:`modelfile_yaml_anchor` with the name "BodyAppearance" for the appearance of the CHASSIS link. Here we call that content with: ::

 appearance: *BodyAppearance

as a YAML alias. By adding "*" to the name set with an anchor, you can reference it as an alias.

.. _modelfile_yaml_offset_position:

Writing Link Relative Position
------------------------------

The TURRET_Y link is modeled as a child link of the CHASSIS link.

To do this, first with: ::

 parent: CHASSIS

we explicitly state that this link's parent link is CHASSIS.

Next, we specify the relative position (offset) of this link from the parent link. This is done with the translation parameter, which for this link is: ::

 translation: [ -0.04, 0, 0.08 ]

This sets this link's origin at a position 5[cm] backward and 8[cm] upward from the CHASSIS link origin. This position is based on the parent link's coordinate system.

To confirm the effect of relative position, let's try removing the translation description. Delete the translation line above or comment it out by adding # at the beginning of the line, then reload the model.

You should find that the turret part that was visible earlier has disappeared. This is because the turret part is also placed at the center of the chassis and is buried inside it. So, turn ON :ref:`basics_sceneview_wireframe` in the scene view. You should see something like this:

.. image:: images/tank_turret_y_0.png

With wireframe display, you can confirm that the turret part is buried inside the chassis.

As you can see, to properly position links, you need the translation description as before. Try changing this value to see what happens.

Note that relative orientation (coordinate system direction) can also be specified using the rotation parameter. rotation is written in the form: ::

 rotation: [ x, y, z, θ ]

This specifies orientation (rotation) with a rotation axis and rotation angle around that axis, where x, y, z specify the unit vector of the rotation axis and θ specifies the rotation angle. When format_version is 2.0, θ is always written in degrees. (Other parameters that specify angles are basically the same.)

An actual example of using this parameter will be introduced later.

Writing the Joint
-----------------

Two links with a parent-child relationship are usually connected by a joint. The TURRET_Y link is also connected to the parent link CHASSIS with a yaw axis joint, allowing the yaw orientation relative to CHASSIS to be changed. Information about this is described by the following parameters of the TURRET_Y link: ::

 joint_type: revolute
 joint_axis: -Z
 joint_range: unlimited
 joint_id: 0

Here we first specify revolute for joint_type. This sets a rotational joint between this link and the parent link. (This is a 1-degree-of-freedom rotational joint, also called a hinge.)

joint_axis specifies the joint axis. For a hinge joint, specify its rotation axis here. You can specify this using the letters X, Y, Z, or as a 3D vector. In either case, the axis direction is described in the link's local coordinate system. Here we specify "-Z" to set the negative Z-axis direction as the rotation axis. When specifying the joint axis as a 3D vector, it would be: ::

 joint_axis: [ 0, 0, -1 ]

With this notation, you can set any orientation as the axis, not just X, Y, or Z axes.
  
Since the Z-axis is usually set vertically upward, including in this model, this joint performs yaw axis rotation. Since the direction is negative Z-axis, positive joint angles correspond to rightward rotation and negative angles to leftward rotation. The joint position is set at this link's origin. From the parent link's perspective, this position is the one set earlier with translation.

Other joint_type options include "prismatic" for linear joints. In this case, joint_axis specifies the linear motion direction.

The joint range of motion is set using joint_range. Here we specify unlimited to have no range restrictions. If you want to set a range, write: ::

 joint_range: [ -180, 180 ]

listing the lower and upper limit values. For rotational joints, angle values are also written in degrees. Here we specify a range from -180° to +180°. If the absolute values of the lower and upper limits are the same, you can also write just the absolute value as: ::

 joint_range: 180

joint_id sets the ID value (integer 0 or greater) assigned to this joint. ID values can be referenced on Choreonoid's interface and used to specify which joint to operate. Robot control programs can also use this value to identify joints. This value is not automatically assigned; appropriate values must be explicitly assigned when creating the model. It's not necessary to assign ID values to all joints. However, since this value is sometimes used as an index when storing joint angles in arrays, it's preferable to assign consecutive values starting from 0 without gaps.

This model has two joints for the turret yaw and pitch axes, so we'll assign joint IDs 0 and 1 respectively.

Checking Joint Operation
------------------------

To check if the joint is modeled correctly, it's effective to actually move the model's joint on Choreonoid's GUI. Let's try this using the functions introduced in :doc:`../index` - :doc:`../pose-editing`.

First, let's move the joint using :ref:`pose_editing_joint_slider_view`. When you select the model being created in the item tree view, the joint displacement view should look like this:

.. image:: images/jointslider0.png

This display shows that a joint named TURRET_Y with joint ID 0 has been defined. Try operating the slider here. You should be able to confirm that the box corresponding to TURRET_Y rotates around the yaw axis in the scene view. For example, the model poses when the joint angle is -30°, 0°, and +30° are as follows:

.. image:: images/tank_turret_y_rotation.png

For TURRET_Y, the joint range is unlimited, so the joint slider can move in the range from -360° to +360°. If range restrictions are applied, the slider can be operated within that range.

:ref:`sceneview_forward_kinematics` is also possible. Switch the scene view to edit mode and drag the TURRET_Y part with the mouse. You should be able to rotate the joint to follow the mouse movement. If it doesn't work well, check the settings on the linked page above.

.. _modelfile_yaml_TURRET_P_description:

Writing the Turret Pitch Axis Part
----------------------------------

Next, let's write the turret pitch axis part. First, add the following under links:

.. code-block:: yaml
 :dedent: 0

   -
     name: TURRET_P
     parent: TURRET_Y
     translation: [ 0, 0, 0.05 ]
     joint_type: revolute
     joint_axis: -Y
     joint_range: [ -10, 45 ]
     max_joint_velocity: 90
     joint_id: 1
     elements:
       - 
         # Turret
         type: RigidBody
         center_of_mass: [ 0, 0, 0 ]
         mass: 3.0
         inertia: [
           0.1, 0,   0,
           0,   0.1, 0,
           0,   0,   0.1 ]
         elements:
           Shape:
             geometry:
               type: Cylinder
               height: 0.1
               radius: 0.1
             appearance: *BodyAppearance

As specified in name, this link is named "TURRET_P". The notation ::

 # Turret

is a comment. Text from # to the end of the line becomes a comment.

After writing this and reloading the model, it should be displayed as follows:

.. image:: images/tank_turret_p.png

The base part for the turret pitch axis has been added.

RigidBody Node
--------------

In the above description, :ref:`modelfile_yaml_rigidbody_parameters` are not written in the Link node but separately using a node called RigidBody.

The RigidBody node is specialized for describing rigid body parameters and can describe the three parameters center_of_mass, mass, and inertia. These have the same meaning as when used in Link nodes. By writing this node under elements of a Link node, you can also set rigid body parameters. Conversely, you can think of the ability to write rigid body parameters directly in Link nodes as a simplified notation replacing RigidBody.

The advantages of deliberately using RigidBody nodes to describe rigid body parameters include:

1. Enables sharing of rigid body parameters
2. Can be written in any coordinate system
3. Can be written as a combination of multiple rigid bodies

First, since rigid body parameters can be written as independent nodes, applying :ref:`modelfile_yaml_anchor` and :ref:`modelfile_yaml_alias` to them enables sharing the same rigid body parameters. This is convenient when modeling mechanisms that use many identical parts.

Also, when nodes are independent, :ref:`modelfile_yaml_transform_node` can be applied individually, making it possible to describe each rigid body's parameters in any coordinate system.

Furthermore, since there's no limit to the number of RigidBody nodes used in each link's description, it's possible to describe the link's overall rigid body parameters as a combination of multiple rigid bodies. In this case, rigid body parameters reflecting all RigidBody nodes contained in the link are set as the link's rigid body parameters. Combining this with advantages 1 and 2 enables efficient and maintainable modeling even for complex shapes composed of multiple parts.

As an example of using RigidBody nodes, the TURRET_P link is composed by combining two RigidBody nodes. The first is the turret pitch axis base part loaded earlier, and the second is the gun barrel part connected to it.

Note that RigidBody is also a node that supports :ref:`modelfile_yaml_elements`, so it can contain other nodes using this. Here we describe the shape part explained below inside elements. This allows the rigid body's physical parameters and shape to be grouped together under the RigidBody node, making the model structure clearer.

.. _modelfile_yaml_turret_pitch_shape:

Writing the Turret Pitch Axis Base Shape
----------------------------------------

The shape of the turret pitch axis base is written as follows: ::

          Shape:
            geometry:
              type: Cylinder
              height: 0.1
              radius: 0.11
            appearance: *BodyAppearance

Here we use a Cylinder node for geometry to represent a cylinder shape. The Cylinder node specifies the cylinder's height and radius as parameters. The cylinder's position and orientation correspond to a shape created by extruding a circle of the specified radius on the XZ plane centered at the origin in both positive and negative Y-axis directions by the height amount. Here we use this orientation without modification.

For appearance, we reference BodyAppearance as an alias as before, using the same settings as before.


Writing the Gun Barrel Part
---------------------------

Let's also add the gun barrel part description. Add the following code to the elements of the TURRET_P link (be careful to match the indentation).

.. code-block:: yaml
 :dedent: 0

       - 
         # Gun
         type: Transform
         translation: [ 0.2, 0, 0 ]
         rotation: [ 0, 0, 1, 90 ]
         elements:
           RigidBody:
             center_of_mass: [ 0, 0, 0 ]
             mass: 1.0
             inertia: [
               0.01, 0,   0,
               0,    0.1, 0,
               0,    0,   0.1 ]
             elements:
               Shape:
                 geometry:
                   type: Cylinder
                   height: 0.2
                   radius: 0.02
                 appearance: *BodyAppearance
		 
When you reload the model, the gun barrel part should also be displayed as follows:
		   
.. image:: images/tank_turret_barrel.png

This part is also written using a RigidBody node as before, with the shape included within this node. The shape also uses a cylinder, with the length and radius adjusted to represent the gun barrel.



.. _modelfile_yaml_transform_node:

Transform Node
--------------

In the gun barrel description, we insert the following above the RigidBody node: ::

 type: Transform
 translation: [ 0.2, 0, 0 ]
 rotation: [ 0, 0, 1, 90 ]
 elements:

This part is called a Transform node.

The Transform node is used to transform the coordinate system of content written under its elements. This has the same effect as the translation and rotation parameters of Link nodes described in :ref:`modelfile_yaml_offset_position`. However, it differs in that it targets nodes written under elements of Link nodes and that multiple Transform nodes can be combined.

To see this effect, let's disable the Transform node. You could remove the entire Transform node, but you can reproduce the same result by commenting out the translation and rotation parts as follows: ::

 type: Transform
 #translation: [ 0.2, 0, 0 ]
 #rotation: [ 0, 0, 1, 90 ]
 elements:

When you reload the model in this state, the result should look like the following figure:

.. image:: images/tank_turret_barrel_no_transform.png

The part buried in the turret is the gun barrel we described. The position is incorrect, and the orientation is sideways.

This is because the coordinate system of the cylinder shape generated by the Cylinder node is originally set this way, and this is a natural result. While this coordinate system was fine for the turret pitch axis base part earlier, when using it as a gun barrel, we need to correct this position and orientation.

That's why we inserted the Transform node above. Here with: ::

 rotation: [ 0, 0, 1, 90 ]

we first rotate 90 degrees around the Z-axis so the gun barrel direction matches the model's front-back direction (X-axis). Then with: ::

 translation: [ 0.2, 0, 0 ]

we move the cylinder forward 20cm to position it at the front of the turret.

Note that the Transform's elements also include the RigidBody node. This means the coordinate transformation above is applied not only to the shape but also to the rigid body parameters described in the RigidBody node. In other words, you can describe the rigid body parameters in the cylinder's local coordinates, which reduces the effort required to calculate the center of mass position and inertia tensor.

.. _modelfile_yaml_transform_parameters:

Transform Parameters
~~~~~~~~~~~~~~~~~~~~

Instead of using Transform nodes, there's also a method to write translation and rotation parameters directly in the target node. These parameters are called "Transform parameters".

For example, since RigidBody nodes also support Transform parameters, the gun barrel part can also be written as follows:

.. code-block:: yaml
 :dedent: 0

       - 
         # Gun
         type: RigidBody:
         translation: [ 0.2, 0, 0 ]
         rotation: [ 0, 0, 1, 90 ]
         center_of_mass: [ 0, 0, 0 ]
         mass: 1.0
         inertia: [
           0.01, 0,   0,
           0,    0.1, 0,
           0,    0,   0.1 ]
         elements:
          Shape:
             geometry:
               type: Cylinder
               height: 0.2
               radius: 0.02
             appearance: *BodyAppearance

We've simply moved the Transform's translation and rotation directly to RigidBody. This makes the description simpler. Internally, the same processing as inserting a Transform node is performed, so consider this a simplified notation method.

Transform parameters are also available for Shape nodes and device-related nodes explained later.


Writing the Turret Pitch Axis Joint
-----------------------------------

Let's also check the turret pitch axis joint description. In the TURRET_P link, the joint is described in the following part: ::

 parent: TURRET_Y
 translation: [ 0, 0, 0.04 ]
 joint_type: revolute
 joint_axis: -Y
 joint_range: [ -45, 10 ]
 joint_id: 1

The parent link is TURRET_Y. The joint is set between this link. Also, translation sets the offset from the parent link to 4cm in the Z-axis direction.

The joint type is specified as revolute like TURRET_Y, making it a rotational (hinge) joint. Here we set the rotation axis to the Y-axis corresponding to the pitch axis. However, the axis direction is negative, making negative joint angles correspond to downward gun barrel rotation and positive angles to upward rotation. Also, joint_range sets the range of motion to 45° upward and 10° downward. joint_id is set to 1, different from the 0 set for TURRET_Y.

Let's also check the behavior of this joint. The joint slider view should display interfaces for two joints corresponding to TURRET_Y and TURRET_P as follows:

.. image:: images/jointslider01.png

Using the slider here or dragging in the scene view, first try moving the pitch axis (TURRET_P). This should allow you to change the vertical orientation of the gun barrel as follows:

.. image:: images/tank_turret_p_rotation.png

Also, the yaw axis behaves the same as before, but you can confirm that the gun barrel's yaw orientation also changes in conjunction. This is because the TURRET_P link is a child link of the TURRET_Y link.


Writing Devices
---------------

In robot models defined in Choreonoid, equipment such as sensors mounted on robots are called "devices". This Tank model will be equipped with two devices: a spotlight and a camera. Below we explain how to write these devices.

.. _modelfile-tank-spotlight:

Writing the Spotlight
~~~~~~~~~~~~~~~~~~~~~

First, let's mount a light (light source) device so we can simulate a robot operating in darkness. There are several types of lights, but here we'll use a spotlight, which is common for lights mounted on robots.

Since devices are mounted on one of the links, write their definitions under the link's elements. Let's mount the light on the turret pitch axis part so we can change the light direction. This way, the light orientation will change in conjunction with the turret yaw and pitch axis movements.

To achieve this, add the following description to the elements of the TURRET_P link:

.. code-block:: yaml
 :dedent: 0

       -
         type: SpotLight
         name: Light
         translation: [ 0.08, 0, 0.1 ]
         direction: [ 1, 0, 0 ]
         beam_width: 36
         cut_off_angle: 40
         cut_off_exponent: 6
         attenuation: [ 1, 0, 0.01 ]

Here, type: SpotLight makes this a SpotLight node description corresponding to a spotlight device. Key points of the description:

* We set "Light" as the name of this device. Since programs that handle devices often access devices by name, please set names for devices like this.
* Device nodes can also use :ref:`modelfile_yaml_transform_parameters`. Here we specify the light installation position with translation. This is relative to the TURRET_P link origin.
* The SpotLight's direction parameter specifies the optical axis direction. Since we want it facing the model's front, we set it to the X-axis direction.
* The beam_width, cut_off_angle, and cut_off_exponent parameters set the spotlight's illumination range. Also, attenuation sets how the light attenuates with distance from the light source.

Writing the Light Shape
~~~~~~~~~~~~~~~~~~~~~~~

Let's write a shape corresponding to the light. Add the following as elements at the end of the SpotLight node:

.. code-block:: yaml
 :dedent: 0

         elements:
           Shape:
             rotation: [ 0, 0, 1, 90 ]
             translation: [ -0.02, 0, 0 ]
             geometry:
               type: Cone
               height: 0.04
               radius: 0.025
             appearance:
               material:
                 diffuse: [ 1.0, 1.0, 0.4 ]
                 ambient: 0.3
                 emissive: [ 0.8, 0.8, 0.3 ]

Here we use a cone shape (Cone node) for the light shape. Since the default coordinate system doesn't have the right orientation, we use :ref:`modelfile_yaml_transform_parameters` to change the orientation. Also, we position it slightly backward so the light source isn't hidden by this shape. This needs attention when generating shadows in rendering.

In material, we also set emissive so the light part appears to glow even in darkness.

After writing this and reloading the model, the light shape should be displayed as follows:

.. image:: images/tank_light.png

This allows you to visually confirm that the light is installed in the correct position and orientation by looking at the model.

However, when mounting devices, corresponding shapes are not necessarily required. Also, even if there is a corresponding shape, it doesn't necessarily have to be written under the device node's elements. We did this in this example to make the modeling clearer, but devices basically function independently of shapes.

.. _modelfile-tank-camera:

Writing the Camera
~~~~~~~~~~~~~~~~~~

Let's also add a camera device. Like the SpotLight node, add the following under the elements of the TURRET_P link:

.. code-block:: yaml
 :dedent: 0

       - 
         type: Camera
         name: Camera
         translation: [ 0.1, 0, 0.05 ]
         rotation: [ [ 1, 0, 0, 90 ], [ 0, 1, 0, -90 ] ]
         format: COLOR_DEPTH
         field_of_view: 65
         near_clip_distance: 0.02
         width: 320
         height: 240
         frame_rate: 30
         elements:
           Shape:
             rotation: [ 1, 0, 0, 90 ]
             geometry:
               type: Cylinder
               radius: 0.02
               height: 0.02
             appearance:
               material:
                 diffuse: [ 0.2, 0.2, 0.8 ]
                 specular: [ 0.6, 0.6, 1.0 ]
                 specular_exponent: 80

Cameras are written using Camera nodes.

In this node, specify the format of images to acquire with format. You can specify one of the following three:

* COLOR
* DEPTH
* COLOR_DEPTH

If you specify COLOR, it becomes a normal color image. For DEPTH, you get a depth image. For COLOR_DEPTH, you can acquire both types of images simultaneously. This is intended for simulating RGBD cameras like Kinect.

Also, specify the image size (resolution) with width and height. Here we set a resolution of 320x240. Furthermore, set the image acquisition frame rate with frame_rate.


Writing Camera Position and Orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The camera position is set slightly below the light with: ::

 translation: [ 0.1, 0, 0.05 ]

For camera orientation, by default the positive Y-axis corresponds to the camera's up direction and the negative Z-axis corresponds to the camera's front (viewing) direction. If you want to point the camera in a different direction, you need to change the camera's orientation using rotation of :ref:`modelfile_yaml_transform_node` or :ref:`modelfile_yaml_transform_parameters`.

In this model, since the Z-axis is taken as vertically upward, the camera would face downward with the default orientation. So we set the desired camera orientation by writing: ::

 rotation: [ [ 1, 0, 0, 90 ], [ 0, 1, 0, -90 ] ]

in the upper Transform node.

The method of specifying orientation with rotation was explained in :ref:`modelfile_yaml_offset_position` as a pair of rotation axis and rotation angle. Here we have two such pairs. Actually, rotation can be written by listing multiple orientation expressions like this. In this case, the orientation values (rotation commands) are applied in order from right to left. (It's the same as applying matrix multiplication in this order, considering each element as a rotation matrix.)

Here we first rotate -90 degrees around the Y-axis with [ 0, 1, 0, -90 ]. This makes the camera face forward. However, in this state the camera's up direction is still the model's left direction, resulting in an image as if the camera were lying on its side. So we further rotate 90 degrees around the X-axis with [ 1, 0, 0, 90 ] to stand the camera up and obtain the desired image.

While it's possible to combine these two rotations into a single rotation expression, such combined values are difficult to intuitively understand or calculate. In contrast, by combining multiple rotations as above, such textual description becomes easier.


Camera Shape
~~~~~~~~~~~~

Here we add a cylinder shape representing the camera lens. This makes the model display look like this:

.. image:: images/tank_camera.png

Note that in the camera definition we have: ::

 near_clip_distance: 0.02

This shifts the range of the external world captured in the camera image slightly forward from the camera center point. Since we've added a camera shape, without this the forward view would be blocked by that shape. By including this description, it becomes possible to capture the area outside the camera shape in the camera image.

.. _modelfile_yaml_crawlers:

Writing the Crawlers
--------------------

Finally, let's write the crawler parts.

Writing the Left Crawler
~~~~~~~~~~~~~~~~~~~~~~~~

Let's start with the left crawler. Returning to the hierarchy (indentation) of :ref:`modelfile_yaml_links` mentioned earlier, add the following description:

.. code-block:: yaml
 :dedent: 0

   -
     name: TRACK_L
     parent: CHASSIS
     translation: [ 0, 0.2, 0 ]
     joint_type: pseudo_continuous_track
     joint_axis: Y
     center_of_mass: [ 0, 0, 0 ]
     mass: 1.0
     inertia: [
       0.02, 0,    0,
       0,    0.02, 0,
       0,    0,    0.02 ]
     elements:
       Shape: &TRACK 
         geometry:
           type: Extrusion
           cross_section: [
             -0.22, -0.1,
              0.22, -0.1,
              0.34,  0.06,
             -0.34,  0.06,
             -0.22, -0.1
             ]
           spine: [ 0, -0.05, 0, 0, 0.05, 0 ]
         appearance:
           material:
             diffuse: [ 0.2, 0.2, 0.2 ]

When you reload the model in this state, the left crawler should be added to the model as follows:

.. image:: images/tank_crawler_l.png

Since crawlers are connected to the chassis, this link specifies CHASSIS as the parent link again.

Also, as the relative position from the parent link, by writing: ::

 translation: [ 0, 0.2, 0]

we set this link's position to the left side of the chassis.

Crawlers are originally mechanisms where belt-like tracks made of connected metal or rubber treads are driven by internal wheels. Simulating such complex mechanisms is generally a difficult task. Therefore, the crawler we model here will be a pseudo crawler represented by a single link. Since it's a single link, there's no belt-like track, and the entire crawler is represented as a single rigid body. While its traversability doesn't match that of accurate crawlers at all, by applying propulsive force to the contact area between the crawler and environment, it's possible to achieve movement somewhat similar to crawlers. For details, see :doc:`../../simulation/pseudo-continuous-track`.

To use a link as such a pseudo crawler (simple crawler), specify "pseudo_continuous_track" for joint_type.

.. note:: Previously, this was set by specifying fixed for joint_type and setting the actuation_mode parameter to "jointSurfaceVelocity". While this notation still works, please use the joint_type setting as above going forward.

For joint_axis, specify the assumed rotation axis direction of the crawler wheels. Forward motion is the positive right-hand screw direction relative to this axis. Here we set the Y-axis as the rotation axis.


The crawler shape is described using an "Extrusion" type geometric shape node. This is also called an extruded shape, where you first specify the cross-sectional shape with cross_section, then describe a 3D shape by extruding it according to the spine description. Here we make the crawler cross-section trapezoidal and extrude it in the Y-axis direction to give it width. This description method was originally defined in VRML97, and for details please refer to the `VRML97 Extrusion node specification <http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Extrusion>`_.

We also perform :ref:`modelfile_yaml_anchor` on the shape described here. We attach the anchor "TRACK" so it can be reused for the right crawler shape.

Writing the Right Crawler
~~~~~~~~~~~~~~~~~~~~~~~~~

Let's also write the right crawler. Add the following to the links hierarchy as before:

.. code-block:: yaml
 :dedent: 0

   -
     name: TRACK_R
     parent: CHASSIS
     translation: [ 0, -0.2, 0 ]
     joint_type: pseudo_continuous_track
     joint_axis: Y
     center_of_mass: [ 0, 0, 0 ]
     mass: 1.0
     inertia: [
       0.02, 0,    0,
       0,    0.02, 0,
       0,    0,    0.02 ]
     elements:
       Shape: *TRACK 

The content of this link is almost the same as the left crawler except for being left-right symmetric. For the shape, we reference the anchor set earlier with the name "CRAWLER" as an alias.

If you reload the model and see a model displayed like below, it's complete!

.. image:: images/tank.png

.. Writing Additional Information
.. -----------------------------

.. Although we've completed writing the model body, it's also possible to write additional supplementary information.

.. This is written the same way as :doc:`modelfile-yaml`. While that explains the method of combining OpenHRP format model files with YAML files, the model files we've worked with in this format are originally written in YAML, so :doc:`modelfile-yaml` can be included directly in the same file.
