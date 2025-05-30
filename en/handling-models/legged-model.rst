Handling Legged Models
======================

Choreonoid includes useful features for manipulating the posture of legged robots such as bipedal humanoid robots. This section introduces these features.

.. contents::
   :local:
   :depth: 1


Sample Model
------------

As a sample for introducing the features, we will use a model of the bipedal humanoid robot "GR001". This model is stored in the "model/GR001" directory in Choreonoid's share directory as a file named "GR001.body". A sample project for handling this model is also provided as "project/GR001Sample.cnoid".

As preparation, please load this project file. You should see the GR001 robot model displayed in the Scene View as shown below.

.. image:: images/GR001Scene.png


Manipulating the Waist with Preset Kinematics
----------------------------------------------

When editing the posture of legged models, there are cases where you want to move the waist position or orientation while keeping the feet grounded in place. If you only need to fix one foot, this can be achieved with normal inverse kinematics (moving the waist with the foot as the base link), but there are often situations where you want to move the waist while keeping multiple feet fixed.

In fact, for legged models, if you move the waist in "Preset Kinematics Mode" (see :ref:`model_kinematics_mode`), it becomes inverse kinematics that moves the waist while keeping multiple feet fixed. This allows you to efficiently edit the waist position and orientation.

Displaying the Center of Mass
------------------------------

When editing the posture of legged models, you may want to check whether the robot is balanced by looking at the position of the overall center of mass. The Scene View allows you to display markers for this purpose.

You can toggle the display from the context menu of the Scene View. Right-click while the mouse cursor is pointing at the target model in edit mode, and the following menu will appear.

.. image:: images/GR001_context_menu.png

Here you can see that there are items called "Center of Mass" and "Center of Mass Projection" in the "Markers" submenu. You can turn on the display by clicking on the item to add a check mark. (Clicking again will remove the check mark and turn off the display.)

Checking "Center of Mass" displays the center of mass position with a green crosshair (the intersecting point is the center of mass position). Also, "Center of Mass Projection" displays the projection point of the center of mass onto the floor surface (the point where Z=0). This is used when checking whether the center of mass is within the foot sole region.

.. _legged_model_zmp:

Zero Moment Point (ZMP)
-----------------------

Similar to the center of mass marker, you can also display a marker representing the "Zero Moment Point (ZMP)" by checking "Markers" - "ZMP" in the context menu. This is a marker consisting of a green sphere and crosshair shown in the figure below, typically positioned near the robot's feet.

.. image:: images/GR001_context_menu_ZMP.png

.. note:: ZMP is the point where the moment received by the foot soles from the floor surface becomes zero (the center of pressure of ground reaction force), and is one of the fundamental concepts in bipedal walking control. In a real robot, ZMP exists within the "support polygon" formed by the convex hull of the foot sole contact area, so this condition can be used in planning target motion trajectories and walking stabilization control. For details, please refer to literature such as `"Humanoid Robot" edited by Shuuji Kajita <https://www.amazon.co.jp/%E3%83%92%E3%83%A5%E3%83%BC%E3%83%9E%E3%83%8E%E3%82%A4%E3%83%89%E3%83%AD%E3%83%9C%E3%83%83%E3%83%88-%E6%94%B9%E8%A8%822%E7%89%88-%E6%A2%B6%E7%94%B0-%E7%A7%80%E5%8F%B8/dp/4274226026/ref=sr_1_1?crid=EEUOQDS14522&dib=eyJ2IjoiMSJ9.R_UAWCVlCn3r8FVqkntUMzdRyiHmskSphNVPShpStvGg1lwBvWm_SP-ufcM1gEKB6HJkYilNE39HvHpxWnYjdeDnE0BFCB2UptC82KVLM66yhukmfZmZLOBrvja8zGcwflg2Hc26XlrcL31tnP3lAdMD9dFZcrbzOaLqFpUSyyG2R9FbrJ3NvYB5YDrvk-TKuiSULT62yhdI66BH9dPWEK_e6c3eNnuNjmDuDAShf88.on_TjkDcBHvhLRQNEabheuSjccKLqrlLpvdfyLoIPDg&dib_tag=se&keywords=%E3%83%92%E3%83%A5%E3%83%BC%E3%83%9E%E3%83%8E%E3%82%A4%E3%83%89%E3%83%AD%E3%83%9C%E3%83%83%E3%83%88&qid=1749546642&s=books&sprefix=%E3%83%92%E3%83%A5%E3%83%BC%E3%83%9E%E3%83%8E%E3%82%A4%E3%83%89%E3%83%AD%E3%83%9C%E3%83%83%E3%83%88%2Cstripbooks%2C152&sr=1-1>`_.

In Choreonoid, the ZMP marker is used for two purposes.

The first purpose is to display the ZMP calculated from motion trajectory data or obtained from actual robot sensor states. This allows you to verify whether the motion trajectory data or actual robot state is normal.

The second purpose is for users to provide a target ZMP position (target ZMP) when editing model postures or motions. In this case, users can arbitrarily set the position of the ZMP marker.

One way to move the ZMP marker is to directly drag it with the mouse on the Scene View. In this case, you can change the 2D position (X, Y coordinates) on the floor surface while keeping the vertical position of the ZMP fixed on the floor surface (Z=0).

Alternatively, for legged models, the following "ZMP Panel" is displayed on the :ref:`model_body_link_view`, which can also be used to change the ZMP position.

.. image:: images/BodyLinkView_ZMP.png

In this case, you can accurately check and change the ZMP position using coordinate values.


.. _model_legged_body_bar:

Legged Body Bar
---------------

The following "Legged Body Bar" is provided as a toolbar that consolidates convenient operations for editing the posture of legged models, including center of mass and ZMP markers.

.. image:: images/LeggedBodyBar.png

.. note:: This toolbar is not displayed by default, so please display it according to the instructions in :ref:`basics_show_toolbar` before using it.

.. |i0| image:: ./images/center-cm.png
.. |i1| image:: ./images/zmp-to-cm.png
.. |i2| image:: ./images/cm-to-zmp.png
.. |i3| image:: ./images/right-zmp.png
.. |i4| image:: ./images/center-zmp.png
.. |i5| image:: ./images/left-zmp.png
.. |i6| image:: ./images/stancelength.png

The functions of each icon are as follows:

.. tabularcolumns:: |p{2.0cm}|p{13.0cm}|

.. list-table::
 :widths: 5,95
 :header-rows: 0

 * - |i0|
   - Moves the center of mass horizontally so that the center of mass projection coincides with the center of both foot soles.
 * - |i1|
   - Moves the center of mass horizontally so that the center of mass projection coincides with the ZMP.
 * - |i2|
   - Sets the ZMP at the position of the center of mass projection.
 * - |i3|
   - Sets the ZMP at the center of the right foot.
 * - |i4|
   - Sets the ZMP at the center of both feet.
 * - |i5|
   - Sets the ZMP at the center of the left foot.
 * - |i6|
   - Adjusts the width between both feet. The width is set in the adjacent numerical input box.

By combining the functions to set the ZMP at the center of the right or left foot with the function to align the center of mass projection with the ZMP, you can also set a posture with the center of mass on either the left or right foot.

Operations Related to Foot Sole Contact
---------------------------------------

When editing the posture of legged robots, it becomes necessary to make the foot soles contact the floor surface. This section explains the key points for performing such operations.

First, when grounding the foot soles or moving them on the floor, you may want to keep the foot sole surface parallel to the floor surface. If the original orientation of the foot sole surface is tilted, you must correct it to be parallel. While you can correct the orientation by directly dragging on the Scene View, it is difficult to accurately match the desired orientation with this method.

If you just need to make the foot sole horizontal, you can execute "Set Level" from the context menu for the foot link on the Scene View. Even if the floor is tilted, you can relatively easily obtain a foot sole orientation parallel to the floor surface by adjusting the roll (R) and pitch (P) values of the link using the :ref:`model_body_link_view`.

Also, when grounding the foot soles, in addition to the foot sole orientation, you need to make the height the same as the floor surface to fit perfectly on the floor surface. To do this, you can use the :ref:`collision_detection_penetration_block` introduced in :doc:`collision-detection`.

First, configure the settings so that collision detection and penetration blocking functions are enabled. Next, adjust the foot sole surface to be parallel to the floor surface using the operations described above. Then lower the foot link toward the floor surface. When the foot sole surface contacts the floor surface, it will no longer be able to move in the direction of the floor surface, so the foot sole surface can be set at a height that almost matches the floor surface. The operation of lowering the foot link toward the floor surface can be done by directly dragging on the Scene View, or by decreasing the Z coordinate value on the Body/Link View. In the latter case, it becomes easier to adjust the horizontal position on the floor surface as well.

Floor Grid and Floor Model
--------------------------

By default, a "floor grid" as shown in the figure below is displayed on the Scene View.

.. image:: images/floorgrid.png

This assumes a floor surface at Z=0 and is provided to make it easier to grasp the position of the floor surface on the Scene View.

However, the floor grid is only for display purposes and is not treated as a floor surface model in internal processing - it is treated the same as if nothing exists. Therefore, collision checking with body models existing in the scene cannot be performed, so the penetration blocking function for the floor surface cannot be used with this alone, and objects will fall through when performing dynamic simulation. When using the floor grid, it is necessary to be aware of this point.

To make the floor surface effective in internal processing, you need to load a model corresponding to the floor surface as a body item. As a floor model, for example, there is a model file "model/misc/floor.wrl" in Choreonoid's share directory. When you load and display this, a blue floor is displayed as shown in the figure below.

.. image:: images/floor_model_grid.png

By introducing such a floor model, functions related to collision with the floor surface become available. The GR001 sample project also loads this floor model.

However, displaying the floor model on the Scene View may make it difficult to edit the robot's posture. This is because, for example, when you want to see the situation of the foot soles from below, they are hidden behind the underside of the floor and cannot be seen, or the mouse cursor points to the floor surface, preventing viewpoint changes from working as desired. In other words, the floor surface can sometimes get in the way of robot operations.

In such cases, you can load the floor model but turn off its display, using only the floor grid to grasp the floor surface. With this setup, collisions with the floor surface are processed while the floor surface does not interfere with operations. This setting is recommended when working primarily on robot posture editing, and the GR001 sample project is also configured this way.