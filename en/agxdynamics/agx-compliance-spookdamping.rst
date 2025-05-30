:orphan:

======================================
Constraint Compliance and Spook Damping
======================================

AGX Dynamics performs numerical calculations by solving equations of motion and constraint conditions simultaneously.
When constraint conditions are not satisfied, they must be satisfied through some method.
AGX Dynamics uses a viscoelastic (spring-damper) model for this purpose.
However, due to the relationship between formulation and numerical calculation, the viscoelastic model is slightly transformed, with the following relationships:

.. code-block:: text

  F = -Kx - Dv   # Spring Damper model
  F = -1/Cx - Dv # Replace K with compliance 1/C
  CF = -x - CDv  # Multiply C^2 at both sides
  CF = -x - hv   # Merge CD to h

  K = 1/C        # Spring Coefficient [N/m] or [Nm/rad]
  C = 1/K        # Compliance [m/N] or [rad/Nm]

  h = CD = D/K   # Spook Damping [s]
  D = hK = h/C   # Damping Coefficient [Ns/m] or [Nm s/rad]
