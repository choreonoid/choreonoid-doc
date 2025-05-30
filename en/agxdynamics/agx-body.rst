Additional Parameters for Body Models
=====================================

When using the AGX Dynamics plugin, the following additional parameters are available for body models.

.. contents::
   :local:
   :depth: 2

Syntax
------

.. code-block:: text

  links:
    -
      name: Arm
      jointCompliance: 1e-8
      jointSpookDamping: 0.0333
      jointMotor: true
      jointMotorCompliance: 1e-8
      jointMotorSpookDamping: 0.0333
      jointMotorForceRange: [ -1000, 1000 ]
      jointRangeCompliance: 1e-8
      jointRangeSpookDamping: 0.0333
      jointRangeForceRange: [ -1000, 1000 ]
      jointLock: true
      jointLockCompliance: 1e-8
      jointLockSpookDamping: 0.0333
      jointLockForceRange: [ -1000, 1000 ]
      convexDecomposition: true
      AMOR: true
      autoSleep: true

  collisionDetection:
    excludeTreeDepth: 3
    excludeLinks: [ ]
    excludeLinksDynamic: [ ]
    excludeLinkGroups:
      -
        name: groupA
        links: [ linkA, linkB, linkC, ... ]
      -
        name: groupB
        links: [ linkZ, linkY, linkX, ... ]
    excludeSelfCollisionLinks: [ linkP ]
    excludeLinksWireCollision: [ linkQ, linkR, ... ]

.. _agx_autosleep:

Parameter Descriptions
----------------------

Link Parameters
~~~~~~~~~~~~~~~

.. list-table::
  :widths: 10,9,4,4,75
  :header-rows: 1

  * - Parameter
    - Default Value
    - Unit
    - Type
    - Description
  * - jointCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - Joint compliance. Smaller values make joints more rigid, larger values make them more flexible.
  * - jointSpookDamping
    - 0.0333
    - s
    - double
    - Joint spook damping
  * - jointMotor
    - false
    - \-
    - bool
    - Enables joint motor. Automatically enabled when ActuationMode is JOINT_TORQUE or JOINT_VELOCITY.
  * - jointMotorCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - Joint motor compliance. Used for velocity control. Smaller values produce higher output to reach target velocity. Larger values may fail to resist external forces (gravity, contact forces) and not reach target velocity.
  * - jointMotorSpookDamping
    - 0.0333
    - s
    - double
    - Joint motor spook damping
  * - jointMotorForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - Maximum and minimum force/torque limits for joint motor
  * - jointRangeCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - Joint range limit compliance. Smaller values produce higher output to maintain joint limits. Larger values may allow exceeding limits due to external forces (gravity, contact forces).
  * - jointRangeSpookDamping
    - 0.0333
    - s
    - double
    - Joint range limit spook damping
  * - jointRangeForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - Maximum and minimum force/torque for joint range limits
  * - jointLock
    - false
    - \-
    - bool
    - Enables joint locking. Used for position control. Automatically enabled when ActuationMode is JOINT_ANGLE.
  * - jointLockCompliance
    - 1e-8
    - m/N or rad/Nm
    - double
    - Joint lock compliance. Smaller values produce higher output to reach target position. Larger values may fail to resist external forces (gravity, contact forces) and not reach target position.
  * - jointLockSpookDamping
    - 0.0333
    - s
    - double
    - Joint lock spook damping
  * - jointLockForceRange
    - [ double_min, double_max ]
    - N or Nm
    - Vec2
    - Maximum and minimum force/torque for joint lock
  * - convexDecomposition
    - false
    - \-
    - bool
    - Enable/disable convex decomposition. Specify true or false.
  * - AMOR
    - false
    - \-
    - bool
    - Merges relatively stationary rigid bodies to reduce solver computational load. Specify true or false. Also requires setting the corresponding property in :doc:`agx-simulator-item`.
  * - autoSleep
    - false
    - \-
    - bool
    - Enable/disable auto sleep. Specify true or false. Removes stationary rigid bodies from solver to reduce computational load. The AutoSleep property in :doc:`agx-simulator-item` must also be set to true.


Collision Detection Settings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
  :widths: 15,7,4,6,75
  :header-rows: 1

  * - Parameter
    - Default Value
    - Unit
    - Type
    - Description
  * - excludeLinksDynamic
    - \-
    - \-
    - string list
    - Disables collision detection for specified links. However, collisions with special objects like wires remain enabled.
  * - | excludeLinkGroups:
      | -
      |   name
      |   links
    - \-
    - \-
    - |
      |
      | string
      | string list
    - | Disables collisions between links registered in the group.
      |
      | Group name (optional)
      | Link names
  * - excludeSelfCollisionLinks
    - \-
    - \-
    - string list
    - Disables self-collision between specified links and the body
  * - excludeLinksWireCollision
    - \-
    - \-
    - string list
    - Disables collision between specified links and AGXWire

The standard Choreonoid :ref:`modelfile_yaml_collision_detection` parameters "excludeTreeDepth" and "excludeLinks" can also be specified.


Convex Decomposition
--------------------

AGX Dynamics includes functionality to decompose triangular mesh shapes into convex shapes.
When convexDecomposition is set to true for a link parameter, convex decomposition of triangular mesh shapes is performed.
Convex decomposition can potentially improve collision detection performance.

.. note::
  Convex decomposition may fail for complex shapes.

.. note::
  Contact points may differ between triangular meshes and convex decomposed shapes, potentially resulting in different collision behavior.

Samples are available at:

* Project file: choreonoid/sample/AGXDynamics/agxConvexDecomposition.cnoid
* Body file: choreonoid/sample/AGXDynamics/vmark.body

When you run the sample, convex decomposition is executed and the shape is composed of multiple convex shapes.

.. image:: images/convexdecomposition.png