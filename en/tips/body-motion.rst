Standard Body Motion File Format
================================

.. contents::
   :local:
   :depth: 2

.. highlight:: YAML

Overview
--------

In Choreonoid, there is data called "Body Motion" that represents the motion trajectory of a :doc:`../handling-models/bodymodel`, and the "Body Motion Item" that stores this data is used for:

* :ref:`simulation-result-item-output`
* Output destination for motions created with the choreography function

and other purposes.

This section explains the standard file format for reading and writing this body motion (Standard Body Motion File Format).

.. _bodymotion-basic-specification:

Basic Specification
-------------------

The standard body motion file is described as a text file with YAML as the base description format.

The file extension is usually ".seq". Although it could be ".yaml" or ".yml" since it is in YAML format, using ".seq" makes it easier to distinguish from other YAML files. The "seq" derives from "Sequence" and is usually pronounced "seek".

Since body motion represents the motion of the entire model, it consists of multiple types of trajectory data such as:

* Link position and orientation trajectories
* Joint angle trajectories
* Other trajectories (such as ZMP trajectories)

In the standard body motion format, such multiple motion trajectory data is stored in a single file by utilizing YAML's hierarchical structure. Each type of motion trajectory data is called a "component". The file format itself allows listing arbitrary types of motion trajectory components.

In body motion, discretized trajectories are described as data consisting of multiple time-series "frames". Frames are associated at regular intervals on the time axis. There are generally two types of values that represent this interval:

* Time step
* Frame rate

The time step corresponds to the time interval between frames, and the frame rate corresponds to the number of frames per unit time. These have an inverse relationship to each other. The unit for time step is usually "seconds". The corresponding unit for frame rate is "frames per second".

In the standard body motion file, the frame interval is specified using the frame rate.

Note that the standard body motion file can also describe frame data that is not necessarily at regular intervals. In this case, instead of specifying the frame rate, the time corresponding to each frame is described for all frames. This will be described later.


Basic Structure
---------------

The standard body motion file is described with the following structure: ::

 # Top node
 type: CompositeSeq
 content: BodyMotion
 formatVersion: 2
 frameRate: 1000
 numFrames: 7261
 components: 
   - 
     # Component 1
     type: MultiSE3Seq
     content: LinkPosition
     numParts: 1
     frameRate: 1000
     numFrames: 7261
     SE3Format: XYZQWQXQYQZ
     frames: 
     # Frame data for Component 1
       - Frame 1 data
       - Frame 2 data
                .
                .
                .
   - 
     # Component 2
     type: MultiValueSeq
     content: JointDisplacement
     numParts: 2
     frameRate: 1000
     numFrames: 7261
     frames: 
     # Frame data for Component 2
       - Frame 1 data
       - Frame 2 data
                .
                .
                .

# The above comments are for explanation and are not normally necessary. Also, the frame data is shown schematically here, and the actual description content is explained below.

Since this is in YAML format, data at the same hierarchy must be written with the same indentation. In the above example, the description of each component must be written with an indentation level below the top level, and the indentation level must be the same.


Top Node
--------

At the top level of the text, a mapping node consisting of the following keys is described.

.. list-table:: Top Level Node
 :widths: 30, 70
 :header-rows: 1

 * - Key
   - Content
 * - type
   - Specify CompositeSeq
 * - content
   - Specify BodyMotion
 * - formatVersion
   - Format version. Specify 2
 * - frameRate
   - Specify the base frame rate (frames per second) for the entire motion
 * - numFrames
   - Specify the number of frames for the entire motion
 * - components
   - Describe components in listing format

The specified values should be written for type and content.

formatVersion is for being able to read old format data even if the description format is changed in the future. The current version explained in this document is 2, so specify 2 here. If formatVersion is set to 1 or if there is no formatVersion description, it is regarded as an old format file. Note that the old format is not explained here.

For frameRate and numFrames, describe values for the entire motion. In practice, they can be specified individually for each component, but it is desirable to unify them throughout the motion, and writing them here allows you to omit the description in each component. Note that numFrames is for information presentation purposes, and the actual number of frames is the number of frame data actually described in each component.

Under components, describe the components that become the actual motion trajectory data. Multiple types of motion trajectory components can be described in listing format.

Component Node
--------------

One component node describes one type of motion trajectory data. The keys commonly used for each component are as follows.

.. list-table:: Common Part of Component Node
 :widths: 30, 70
 :header-rows: 1

 * - Key
   - Content
 * - type
   - Specify the data type of the motion trajectory as a string
 * - content
   - Specify the purpose of the data as a string
 * - numParts
   - Number of elements per frame. Valid for Multi-type data formats
 * - frameRate
   - Frame rate. If omitted, the value specified in the top node is used
 * - numFrames
   - Specify the number of frames
 * - frames
   - List frame data as a YAML sequence

Specify the data type for type and the purpose of the data for content as strings. Currently, the following types are available.

.. list-table:: Component types
 :widths: 15, 50, 35
 :header-rows: 1

 * - type
   - Data Type
   - Example of content
 * - MultiValueSeq
   - Time series data of frames consisting of multiple floating point values
   - JointDisplacement (joint displacement trajectory)
 * - MultiSE3Seq
   - Time series data of frames consisting of multiple SE(3) values (position and orientation in 3D space)
   - LinkPosition (link position and orientation trajectory)
 * - Vector3Seq
   - Time series data of frames consisting of a single 3D vector value
   - ZMP (zero moment point trajectory)

It is desirable to match the frameRate to the value described in the top node. Alternatively, if there is a description in the top node, the description can be omitted for each component, in which case the value from the top node is applied.

The relationship with the top frame value for numFrames is the same as for frameRate. However, the actual number of frames is determined by the number of frame data described under frames. numFrames is used only for information presentation.

The details of each type and content are explained below.

.. _bodymotion-multivalueseq-type:

MultiValueSeq Type
~~~~~~~~~~~~~~~~~~

This is time series data of frames consisting of multiple floating point values. You can think of it as each frame being composed of multiple scalar values, or as being composed of a single multi-dimensional vector value. Of course, both are the same thing.

A specific use of this type is joint angle trajectories. In that case, specify "JointDisplacement" for content. The reason for using Displacement instead of Angle is that some joints may be prismatic joints rather than revolute joints. JointDisplacement applies to both, and below we will use the term "joint displacement" to include joint angles.

Currently, the only MultiValueSeq type content supported by body motion is JointDisplacement, but you may store any content in the file format. When reading as body motion in Choreonoid, components other than JointDisplacement will be ignored, but there is no problem with other software using other content.

The MultiValueSeq type is a type with "Multi" at the beginning, and in this case the "numParts" of the component node is valid. You need to specify the number of elements (dimensions) per frame there. When used as a joint displacement trajectory, the number of joints is described in numParts.

Under frames, each frame is described as one YAML sequence, containing numParts number of values. The order is according to joint ID order. For revolute joint angles, the unit is radians, and for prismatic joints, it is meters.

An example description of this component is shown below.

.. code-block:: yaml
 :dedent: 1

    - 
      type: MultiValueSeq
      content: JointDisplacement
      numParts: 2
      frameRate: 100
      numFrames: 100
      frames: 
        - [ 0.0,  0.0  ]
        - [ 0.01, 0.01 ]
        - [ 0.01, 0.02 ]
        - [ 0.02, 0.03 ]
        - [ 0.02, 0.04 ]
                .
                .
                .

This is an example with two joints. Although frames after the 6th are omitted here, in reality, 100 frames as specified in numFrames would be written.

.. _bodymotion-multise3seq-type:

MultiSE3Seq Type
~~~~~~~~~~~~~~~~

This is time series data of frames consisting of multiple SE(3) values. SE(3) values represent both position and orientation (rotation) in 3D space.

A specific use of this type is link position and orientation trajectories. In this case, specify "LinkPosition" for content.

For single-link models, trajectory data of this type is needed to represent its motion. Also, for models consisting of multiple links, while joint motion can be represented by JointDisplacement data, to represent the motion of the entire model, the position and orientation trajectory of the root link is still necessary. Therefore, body motion usually includes the position and orientation trajectory data of the root link.

The number of links to include in one frame is specified with numParts, similar to the MultiValueSeq type. The ordering follows the link index order (depth-first traversal order of the link tree). Usually, the first element corresponds to the root link.

SE(3) is a 6-dimensional value combining position and orientation. For the 3 dimensions corresponding to orientation, there are various representation methods such as rotation matrices, quaternions, and roll-pitch-yaw. It is also necessary to decide how to arrange these elements. In MultiSE3Seq type components, this is specified with the "SE3Format" key. The symbols that can be specified for this are summarized below.

.. list-table:: SE3Format Types
 :widths: 20, 80
 :header-rows: 1

 * - Symbol
   - Content
 * - XYZQWQXQYQZ
   - Describe orientation with quaternion. After position X, Y, Z, arrange quaternion W, X, Y, Z values
 * - XYZQXQYQZQW
   - Describe orientation with quaternion similar to XYZQWQXQYQZ, but arrange quaternion in X, Y, Z, W order
 * - XYZRPY
   - Describe orientation in roll-pitch-yaw format. After position X, Y, Z, arrange orientation R, P, Y values

In all cases, one SE(3) value is described as one YAML sequence. The standard format is "XYZQWQXQYQZ". In this format, for example, a value with position (X, Y, Z) of (1, 2, 3) and orientation quaternion (W, X, Y, Z) of (1, 0, 0, 0) is described as ::

 [ 1, 2, 3, 1, 0, 0, 0 ]

Then, such SE(3) values are arranged under frames as YAML sequences for numParts.

An example description of this component is shown below.

.. code-block:: yaml
 :dedent: 1

   - 
     type: MultiSE3Seq
     content: LinkPosition
     numParts: 1
     frameRate: 100
     numFrames: 100
     SE3Format: XYZQWQXQYQZ
     frames: 
       - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
       - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
       - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
       - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
       - [ [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
                  .
                  .
                  .

As in this example, even when numParts is 1, the YAML sequence for each frame is in a doubly nested state. When numParts is 2 or more, in the description of each frame, multiple SE(3) values are arranged as follows. ::

- [ [ X1, Y1, Z1, QW1, QX1, QY1, QZ1 ], [ X2, Y2, Z2, QW2, QX2, QY2, QZ2 ], ... , [ Xn, Yn, Zn, QWn, QXn, QYn, QZn ] ]

* Here, labels like Xn represent numerical values corresponding to each element of the nth SE(3) value. Also, "..." actually contains the 3rd through (n-1)th SE(3) values.

Vector3Seq Type
~~~~~~~~~~~~~~~

This is time series data of frames consisting of a single 3D vector value.

Specific uses of this type include center of gravity trajectories and zero moment point (ZMP) trajectories.

Currently, ZMP is officially supported by BodyMotion, in which case specify "ZMP" for content. Also, if you specify "true" for the "isRootRelative" key, the coordinate system becomes relative coordinates from the root link. If this key is not specified or false is specified, it is described in global coordinates.

Since this type is limited to one value per frame, the numParts specification is invalid.

An example description of this component is shown below.

.. code-block:: yaml
 :dedent: 1

   - 
     type: Vector3Seq
     content: ZMP
     frameRate: 100
     numFrames: 100
     frames: 
       - [ 0.0, 0.0,   0.0 ]
       - [ 0.0, 0.001, 0.0 ]
       - [ 0.0, 0.002, 0.0 ]
       - [ 0.0, 0.003, 0.0 ]
       - [ 0.0, 0.004, 0.0 ]
               .
               .
               .

Combined Use of Link Position/Orientation Trajectories and Joint Displacement Trajectories
------------------------------------------------------------------------------------------

The basis for model motion is link position and orientation trajectories (LinkPosition data of MultiSE3Seq type) and joint displacement trajectories (JointDisplacement data of MultiValueSeq type).

For rigid body models consisting of a single link, there are no joints, so only link position and orientation trajectories with numParts of 1 are needed. On the other hand, for models consisting of multiple links with joints, it is necessary to appropriately combine link position and orientation trajectories with joint displacement trajectories to represent motion. The following forms can be considered:

1. Link position and orientation trajectories for all links
2. Root link position and orientation trajectory + joint displacement trajectories for all joints
3. Link position and orientation trajectories for all links + joint displacement trajectories for all joints

Form 1 represents all motion with link position and orientation data. This allows complete representation of motion when each link is a rigid body.

Form 2 provides link position and orientation only for the root link, and additionally provides joint displacements for all joints. In this case, the position and orientation of links other than the root link can be obtained through forward kinematics calculations using joint displacements. This is the standard format for representing robot motion data. The advantage is that the data size is much smaller compared to form 1. This is because representing the position and orientation of one link requires a 6-dimensional SE(3) value, while one joint can be represented with a 1-dimensional floating point value. Also, it is common to want to reference joint displacements in robots, and this form allows direct access to them. On the other hand, the disadvantage is that link positions and orientations obtained through forward kinematics may deviate from the actual robot or simulation results. This is because the rigidity of links and joints may not be sufficient in actual machines, and simulations may attempt to reproduce this or may have slight deviations due to joint constraint calculation methods.

Form 3 has the advantage of no deviation in link position and orientation while also allowing direct reference to joint displacements. However, the data size becomes the largest among the above combinations.

Since the standard body motion file format supports all the above combinations, please describe data in the appropriate combination for your purpose while considering the advantages and disadvantages of each form.


Description Method for Variable Interval Frame Data
---------------------------------------------------

As mentioned in :ref:`bodymotion-basic-specification`, Choreonoid's body motion assumes that motion frames are arranged at regular time intervals.

However, for some motion data, arranging at regular intervals may not be appropriate. For example, when robot states are output as logs and recorded, the logs from the robot may not be output at regular intervals. Depending on the processing status of control and communication in the robot's computer, there may not be time to spend on log processing, and as a result, it is not uncommon for log output intervals to vary. Also, when the robot is stationary for long periods, holding data at fine time intervals while stationary would be wasteful.

In such cases, it is common to not necessarily have regular frame intervals and instead attach time timestamps to each frame. We will call such data "variable interval frame data" or "frame data with time specification" here.

The standard body motion file can actually describe data in this format as well.

In that case, add the following description to the node: ::

 hasFrameTime: true

Like other parameters, this can be described in each component node, or if described in the top node, the description can be omitted in each component.

Then, at the beginning of each frame data, describe a numerical value corresponding to the time of that frame.

For example, converting the example in :ref:`bodymotion-multivalueseq-type` to variable interval frames would look like this:

.. code-block:: yaml
 :dedent: 1

    - 
      type: MultiValueSeq
      content: JointDisplacement
      numParts: 2
      numFrames: 100
      hasFrameTime: true
      frames: 
        - [ 0.0, 0.0,  0.0  ]
        - [ 0.1, 0.01, 0.01 ]
        - [ 0.3, 0.01, 0.02 ]
        - [ 0.4, 0.02, 0.03 ]
        - [ 0.7, 2,    0.04 ]
                .
                .
                .

Here, the first of the numerical values arranged under frames corresponds to the time. Here, times of 0.0, 0.1, 0.3, 0.4, 0.7 are set. It is OK to specify times that are not at regular intervals like this. However, please specify the time of each frame to be greater than the previous frame. Description methods where time goes backward are not supported by this format.

As a result, the number of numerical values written in each frame is 3, but please note that the number of frame elements is still 2 as written in numParts.

Converting the example given in :ref:`bodymotion-multise3seq-type` to variable intervals with the same times as above would look like this:

.. code-block:: yaml
 :dedent: 1

   - 
     type: MultiSE3Seq
     content: LinkPosition
     numParts: 1
     numFrames: 100
     hasFrameTime: true
     SE3Format: XYZQWQXQYQZ
     frames: 
       - [ 0.0, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
       - [ 0.1, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
       - [ 0.3, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
       - [ 0.4, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
       - [ 0.7, [ -2, -0.5, 0.1, 1, 0, 0, 0 ] ]
                  .
                  .
                  .

In this case, the SE(3) value itself is one sequence, and the time value is written one level above, so please note this point.

Importing Variable Interval Frame Data
--------------------------------------

To reiterate, Choreonoid's body motion is designed for regular interval frame data as mentioned in :ref:`bodymotion-basic-specification`, and cannot store variable interval frame data. Also, there are currently no other data structures in Choreonoid that support variable interval frame data. Therefore, variable interval frame data cannot be read directly into Choreonoid as is.

However, it is possible to import variable interval frame data files as regular interval frame data. In this case, each frame from the source is applied to the nearest destination frame after its time, and that value is maintained until the next destination frame.

.. note:: Interpolation between frames during import would allow for smoother motion trajectory import, but currently there is no such function, and it only sets the original data values to the corresponding frames.

For this function, you first need to determine the frame rate of the destination. Currently, there is no function in Choreonoid's interface to set this, and the frameRate value described in the body motion file is used. Therefore, please describe the desired value for frameRate in the file, at least in the top node.

The import itself can be performed in the same way as reading a normal body motion file. That is, select "Body Motion" from "File" - "Open" in the main menu, and select the file you want to import in the file dialog. If the file contains variable interval frame data, it will automatically be processed as an import.


About Description Style
-----------------------

In YAML, there are two ways of writing: block style and flow style. When describing hierarchical structures, block style uses indentation, while flow style uses curly braces "{ }" (for mapping) and square brackets "[ ]" (for sequences).

In files of this format, you may use either or combine them in any way, but generally it is written as in the above examples. That is, flow style is used for describing each frame's data, and block style is used for other parts. Writing in this way makes it well-organized and easy to read. When outputting files from Choreonoid, this style is used.

Note that YAML is a superset of JSON format, and if everything is written in flow style, it becomes a JSON format file. If you want to handle motion data as JSON format files, please do so. (However, please note that you cannot include comments in JSON.)

Sample Files
------------

There are several standard body motion files for the SR1 model under "motion/SR1" in the share directory (see :doc:`../install/directories`). These are used as motion pattern data loaded from controllers in samples such as "SR1Walk.cnoid" and "SR1WalkinHouse.cnoid".

You can also check what standard body motion files look like by saving body motion items generated as :ref:`simulation-result-item-output`. Select the generated body motion item and execute "File" - "Save selected items as" from the main menu. A file save dialog box will be displayed, which you can use to save the file.