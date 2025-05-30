AGXSimulator Item
=======================

The following additional properties are available in the AGXSimulator item.

.. .. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
  :widths: 10,9,4,4,75
  :header-rows: 1

  * - Parameter
    - Default Value
    - Unit
    - Type
    - Description
  * - NumThreads
    - 1
    - threads
    - unsigned int
    - Number of threads used by AGX Dynamics. Parallelizes internal calculations (collision detection, solver). You can verify if it's working by checking CPU usage with tools like top.
  * - ContactReduction
    - true
    - \-
    - bool
    - Enable/disable contact reduction feature. Specify true or false. Reduces solver computational load by eliminating unnecessary contact points.
  * - ContactReductionBinResolution
    - 3
    - bins
    - unsigned int
    - Number of bins for contact reduction. Specify 1-10. Number of bins used in the 6-dimensional bin packing algorithm.
  * - ContactReductionThreshold
    - 12
    - points
    - unsigned int
    - Threshold for starting contact reduction. When contact points between links exceed the specified threshold, contact reduction begins.
  * - ContactWarmstarting
    - false
    - \-
    - bool
    - When contact state is unchanged from the previous step, uses the previous solver solution to accelerate convergence calculations.
  * - AMOR
    - false
    - \-
    - bool
    - Merges stationary rigid bodies together to reduce solver computational load. Specify true or false. Each link must also be configured. See :doc:`agx-body` for details.
  * - AutoSleep (deprecated)
    - false
    - \-
    - bool
    - Removes stationary rigid bodies from the solver to reduce computational load. Specify true or false. Each link must also have autoSleep configured. See :doc:`agx-body` for details.
  * - SaveToAGXFileOnStart
    - false
    - \-
    - bool
    - Saves the scene in AGX Dynamics file format (.agx) when simulation starts. Save location is the directory where Choreonoid's executable binary is located or the current directory at runtime. Can be used for debugging and performance verification with AGX Dynamics standalone.
  * - DebugMessageOnConsole
    - warning
    - \-
    - debug, info, warning, or error
    - Sets the type of messages output by AGXSimulator. debug: Displays AGX debugging messages. info: Shows forces applied to agxBreakableJoint, etc. warning: When shapes are incorrect, materials are missing from material table, etc. error: Fatal errors when simulation stops or crashes.