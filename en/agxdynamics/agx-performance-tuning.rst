
Performance Tuning
==================

If simulation execution speed is slow or behavior is unstable, try adjusting with the following references.

Improving Calculation Time
--------------------------

* Reduce the number of objects (rigid bodies, joints)
* Use primitive shapes (Box, Sphere, Cylinder, etc)
* Avoid using triangle meshes as much as possible (collision detection becomes slow due to brute-force checking)
* Increase time step
* Increase number of threads
* When using direct solver in friction model, increase surfaceViscosity
* Use Contact Warmstarting
* Use AMOR


Improving Collision Response Stability
--------------------------------------

* Set mass parameters appropriately
* Avoid using extremely small shapes
* Use primitive shapes (Box, Sphere, Cylinder, etc)
* Avoid using triangle meshes as much as possible (collision response tends to be unstable)


Improving Control Stability
---------------------------

* Set mass parameters appropriately
* Joints are less likely to oscillate with velocity or position/angle specification than torque specification (physics engine absorbs integration errors)