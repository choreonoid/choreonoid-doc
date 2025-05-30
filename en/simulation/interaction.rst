Interaction Functions
=====================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

What are Interaction Functions?
--------------------------------

There are times when you want to interact with robots and objects in the virtual world during simulation, such as applying forces or moving them. Choreonoid has functions that allow users to perform such interactions interactively through the GUI, which we call "interaction functions".

How to Operate
--------------

Interaction operations are performed on the scene view.

The following operations become available when:

* A simulation is in progress
* The scene view is in edit mode


Pulling Operation
~~~~~~~~~~~~~~~~~

This is an operation to apply force to a model. When you drag a part where you want to apply force with the left mouse button on the scene view, a pulling force is applied in the direction of the drag. The magnitude of the force is proportional to the length of the drag. Think of it as pulling with a virtual rubber band.

During this operation, a line connecting the point of force application and the drag destination is displayed, which you can think of as a rubber band to help visualize how the force is being applied.

This operation is shown in the following figure:

.. image:: images/interaction-force.png

Here, we first position the mouse cursor on the robot's head and start dragging from there. In this case, we're dragging to the right, and you can see a line extending to the drag destination. As a result, the robot is being pulled in this direction and is about to fall over.

You can change the scale of the force magnitude by combining keyboard operations. If you drag while holding the Shift key, the force magnitude becomes 10 times larger. Furthermore, if you drag while holding Ctrl + Shift, the force magnitude becomes 100 times the original. If the model doesn't move well when you try to drag it, try these operations.

Note that the base force magnitude is automatically determined from the model's mass, but it may be too small or too large for the force you want to apply. Currently, users cannot arbitrarily set this value, but we plan to improve this in the future.


Forced Movement and Holding Operations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

These operations are only valid for a model's root link. When you right-click on the root link of the model you want to move in the above state, the following menu items appear:

* Move forcibly
* Hold forcibly

(See the figure below)

.. image:: images/interaction-move1.png

When you select "Move forcibly", this item becomes checked and enters forced movement mode. When you drag the root link in this state, you can forcibly move the model ignoring physical laws. For example, you can move the robot floating above the floor as shown below:

.. image:: images/interaction-move2.png

Once you click on the root link, a drag marker is also displayed, which you can drag as well. In that case, you can also rotate the model.


In this operation, when you finish dragging, the behavior returns to following physical laws. For example, if you were dragging the robot to float above the floor, when you release the drag, the robot will fall to the floor.

"Hold forcibly" allows similar operations to "Move forcibly". However, the behavior when dragging ends is different, and the root link position remains held in the dragged state. To release the held state, simply uncheck the menu item to exit forced holding mode.


About Synchronization Between Simulation and Display
----------------------------------------------------

As mentioned at the beginning, interaction operations are only valid for ongoing simulations. They cannot be applied during playback of completed simulation results or when checking choreographed motions. You need to be aware of whether the currently displayed model behavior is from an ongoing simulation or from other functions.

Furthermore, the simulation progress and its display need to be synchronized. Since user interaction operations are performed on the scene view display and that content is fed back to the physics calculations inside the simulator, if there is a timing discrepancy between them, you won't be able to operate as intended. Normally they are synchronized, but this is not the case when "Sync with ongoing update" in the time bar is turned off. For details, see :ref:`simulation_playback_ongoing_simulation`.