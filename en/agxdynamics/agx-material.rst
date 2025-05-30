Additional Parameters for Physical Materials
============================================

When using the AGX Dynamics plugin, the following physical material properties are available.

.. contents::
   :local:
   :depth: 2

Sample
------

A sample demonstrating AGX Dynamics Plugin materials is available at:
You can see how different parameter values affect the simulation behavior.

* choreonoid/samples/AGXDynamics/agxMaterialSample.cnoid

Material Configuration Procedure
--------------------------------

In AGXSimulator, friction coefficients and restitution coefficients between links can be adjusted using the following procedure:

1. Define Material and ContactMaterial in a material file
2. Set the Material defined in the material file in the body file

.. _agx_material_file:
   
Material File
-------------

A material file is a list file that describes physical properties such as friction coefficients and restitution coefficients. This file can describe contact properties (ContactMaterial) for the same or different materials. By specifying the material name defined here in the body file, you can set materials for your models.

The material file is loaded by setting it in the world item properties. By default, ``choreonoid/share/default/materials.yaml`` is set and loaded automatically.

.. code-block:: yaml

  materials:
    -
      name: Ground
      roughness: 0.5
      viscosity: 0.0
    -
      name: agxMat5
      density: 1.0

  contactMaterials:
    -
      materials: [ Ground, agxMat5 ]
      youngsModulus: 1.0e5
      restitution: 0.1
      spookDamping: 0.08
      friction: 0.416667
      surfaceViscosity: 1.0e-8
      adhesionForce: 100
      adhesivOverlap: 0.2
      frictionModel: [ iterative, direct ]
      contactReductionMode: reduceGeometry
      contactReductionBinResolution: 3


Material Parameter Descriptions
-------------------------------

Bulk Material
~~~~~~~~~~~~~

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - Parameter
    - Default Value
    - Unit
    - Type
    - Description
  * - density
    - 1000
    - kg/m³
    - double
    - Density. Used for automatic calculation of link mass, inertia tensor, and center of mass.
  * - youngsModulus
    - 4.0e8
    - Pa
    - double
    - Young's modulus. Represents the stiffness of links (rigid bodies). Lower values make links more prone to interpenetration.
  * - viscosity
    - 0.5
    - \-
    - double
    - Restitution viscosity. Pairs of restitution viscosity values determine the restitution coefficient.
  * - spookDamping
    - 0.075
    - s
    - double
    - Spook damping. Used to relax interpenetration between links (satisfying constraint conditions).
  * - poissonRatio
    - 0.3
    - \-
    - double
    - Poisson's ratio (deprecated since 2.27.0.0)

Surface Material
~~~~~~~~~~~~~~~~

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - Parameter
    - Default Value
    - Unit
    - Type
    - Description
  * - roughness
    - 0.5
    - \-
    - double
    - Surface roughness. Pairs of surface roughness values determine the friction coefficient.
  * - surfaceViscosity
    - 5e-09
    - \-
    - double
    - Surface viscosity. Viscosity acting in tangential directions. Pairs of surface viscosity values become the ContactMaterial's surfaceViscosity. Used to represent wetness like oil.
  * - adhesionForce
    - 0.0
    - N
    - double
    - Adhesion force. When shapes are in contact, adhesion force acts in the normal direction. Used for adhesive-like behavior.
  * - adhesivOverlap
    - 0.0
    - m
    - double
    - Adhesion effective distance. Adhesion force becomes active when link penetration > effective distance.

.. note::
  For materials with defined ContactMaterial, the ContactMaterial parameters are used. Surface material parameters from Material are not used.

.. _agx_wire_material:

Wire Material
~~~~~~~~~~~~~

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - Parameter
    - Default Value
    - Unit
    - Type
    - Description
  * - wireYoungsModulusStretch
    - 6e10
    - Pa
    - double
    - Tensile Young's modulus
  * - wireSpookDampingStretch
    - 0.075
    - s
    - double
    - Tensile spook damping
  * - wireYoungsModulusBend
    - 6e10
    - Pa
    - double
    - Bending Young's modulus
  * - wireSpookDampingBend
    - 0.075
    - s
    - double
    - Bending spook damping

.. _agx_contact_material_parameters:

ContactMaterial Parameter Descriptions
--------------------------------------

.. list-table::
  :widths: 10,7,4,4,75
  :header-rows: 1

  * - Parameter
    - Default Value
    - Unit
    - Type
    - Description
  * - youngsModulus
    - 2.0e8
    - Pa
    - double
    - Young's modulus
  * - restitution
    - 0.0
    - \-
    - double
    - Restitution coefficient. 0: perfectly inelastic collision, 1: perfectly elastic collision
  * - spookDamping
    - 0.075
    - s
    - double
    - Spook damping
  * - friction
    - 0.5
    - \-
    - double
    - Friction coefficient
  * - secondaryFriction
    - -1.0
    - \-
    - double
    - Secondary direction friction coefficient. Enabled when secondaryFriction>=0 and friction model is set to oriented_box, oriented_scaled_box, constant_normal_force_oriented_box, or oriented_iterative.
  * - surfaceViscosity
    - 1.0e-8
    - \-
    - double
    - Surface viscosity coefficient. Compliance for friction constraints.
  * - secondarySurfaceViscosity
    - -1.0
    - \-
    - double
    - Secondary direction surface viscosity coefficient. Enabled when secondaryFriction>=0 and friction model is set to oriented_box, oriented_scaled_box, constant_normal_force_oriented_box, or oriented_iterative.
  * - adhesionForce
    - 0.0
    - N
    - double
    - Adhesion force
  * - adhesivOverlap
    - 0.0
    - m
    - double
    - Adhesion effective distance
  * - frictionModel
    - [ default, default ]
    - \-
    - | string
      | string
    - | Friction model: default(iterative), iterative, box, scaled_box, oriented_box, oriented_scaled_box, constant_normal_force_oriented_box, oriented_iterative
      | Solver: default(split), split, direct, iterative, direct_and_iterative

  * - contactReductionMode
    - default
    - \-
    - string
    - Contact reduction mode: default(reduceGeometry), reduceGeometry, reduceALL, reduceNone
  * - contactReductionBinResolution
    - 0
    - \-
    - uint8_t
    - Contact reduction bin resolution. Uses AGXSimulator item parameter when 0.
  * - primaryDirection
    - [ 0, 0, 0 ]
    - Unit vector
    - Vec3
    - Primary direction vector when using orientedBox friction model

  * - referenceBodyName
    - \-
    - \-
    - string
    - Reference body name when using orientedBox friction model
  * - referenceLinkName
    - \-
    - \-
    - string
    - Reference link name when using orientedBox friction model

.. note::
  AGX Dynamics does not distinguish between dynamic and static friction coefficients. In practice, the difference is only 10-20%, which is negligible in most situations.

.. note::
  Additional friction models have been added since Choreonoid 1.7. The iterative and constant_normal_force_oriented_box models correspond to the cone and orientedBox models used up to version 1.7.

.. _not_defined_contact_material:

When ContactMaterial is Not Defined
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ideally, all material pair properties should be described in ContactMaterial, but this can be difficult.
When ContactMaterial is not defined, parameter values are calculated according to the following formulas using parameters described in Material.
Default values are applied when parameters are not set in Material either.

* youngsModulus = (m1.youngsModulus * m2.youngsModulus)/(m1.youngsModulus + m2.youngsModulus)
* restitution = sqrt((1-m1.viscosity) * (1-m2.viscosity))
* spookDamping = max(m1.spookDamping, m2.spookDamping)
* friction = sqrt(m1.roughness * m2.roughness)
* surfaceViscosity = m1.surfaceViscosity + m2.surfaceViscosity
* adhesionForce = m1.adhesionForce + m2.adhesionForce


Material Description in Body Files
----------------------------------

This section explains how to describe materials in body files.
Center of mass, mass, and inertia can be either directly specified or automatically calculated using density, selectable via massType.
The default is mass.

.. code-block:: yaml

  massType: mass             # Direct specification
  massType: density          # Automatic calculation using density

Materials can be selected from those defined in the material file or directly specified.
The default is Default or default as defined in the material file.

.. code-block:: yaml

  material: Default          # Default material
  material: Ground           # Material
  material: useLinkInfo      # Direct specification

Below are description examples.

.. note::
  Currently, calculation results for center of mass, mass, and inertia tensor using density are held internally in AGX Dynamics and cannot be retrieved or confirmed from Choreonoid links or GUI.

Traditional Notation
~~~~~~~~~~~~~~~~~~~~

* Traditional Choreonoid notation
* Uses the described centerOfMass, mass, and inertia
* Material becomes default except for density
* ContactMaterial becomes default vs xxxxx

.. code-block:: yaml

  links:
    -
      name: box1
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]

Using Material File (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Uses parameters described in the material file including density

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density     # Automatically calculate center of mass, mass, and inertia tensor using density
      material: steel       # Use steel from material file
      density: 1.0          # If density is described, it overrides steel's density
                            # and uses the directly specified value

Traditional Notation + Material List (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Uses directly described center of mass, mass, and inertia tensor with massType: mass
* Other material parameters use steel from the material file

.. code-block:: yaml

  links:
    -
      name: box1
      massType: mass      # Use directly described center of mass, mass, and inertia tensor
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]
      material: steel     # Use steel from material file


Direct Description (Not Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* material: useLinkInfo allows using Material parameters described in the body file
* ContactMaterial values are calculated according to :ref:`not_defined_contact_material`

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density
      material: useLinkInfo
      density: 1.0
      youngsModulus:
      poissonRatio:
      viscosity:
      spookDamping:
      roughness:
      surfaceViscosity:
      adhesionForce:
      adhesivOverlap:


Full Description (Not Recommended)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* When everything is described
* Not recommended as it's difficult to determine which parameters are being used

.. code-block:: yaml

  links:
    -
      name: box1
      massType: density               # Automatically calculate center of mass, mass, and inertia tensor using density
      centerOfMass: [ 0, 0, 0 ]
      mass: 1.0
      inertia: [
        0.02, 0,    0,
        0,    0.02, 0,
        0,    0,    0.02 ]
      material: steel                 # Use material list
      density: 1.0                    # Use described density
      youngsModulus:                  # Following are not used
      poissonRatio:
      viscosity:
      spookDamping:
      roughness:
      surfaceViscosity:
      adhesionForce:
      adhesivOverlap: