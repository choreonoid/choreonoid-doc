========
Usage
========

.. contents::
   :local:
   :depth: 1

How to Execute Simulations
--------------------------

Simulations using the AGX Dynamics plugin can be realized in the same way as when using :ref:`other physics simulators<simulation_creation_and_configuration_of_simulator_item>`. By creating the simulator item "AGXSimulator" and placing it as a child item of the world item, execution becomes possible.

Creating and Setting AGXSimulator Item
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Please create it by selecting "File" - "New" - "AGXSimulator" from the main menu.
After creation, place the AGXSimulator item under the world item in the item tree view.
The AGXSimulator item has properties that other simulator items do not have.
For details, please see :doc:`agx-simulator-item`.

Executing Simulations
~~~~~~~~~~~~~~~~~~~~~

Start the simulation by pressing the simulation execution button with AGXSimulator selected in the item tree view.

Specialized Models Available with AGX Dynamics Plugin
-----------------------------------------------------

With the AGX Dynamics plugin, you can use specialized models such as crawlers and wires.
Please check the details from the table of contents on the left.

Samples
-------

Sample projects using the AGX Dynamics plugin are located in choreonoid/samples/AGXDynamics.
Please try executing them.