Sample Controllers
==================

The task simulation samples provide controllers for operating robots with gamepads. This section explains the operation methods for each controller.

.. contents::
   :local:

Supported Gamepads
^^^^^^^^^^^^^^^^^^
Currently supported gamepads include:

* Sony PlayStation 4 gamepad (DualShock 4)
* Sony PlayStation 3 gamepad (DualShock 3)
* Logitech F310
* Microsoft Xbox Controller
* Microsoft Xbox 360 Controller

If you already own any of the above, please use it. If you need to purchase a new one, we recommend the PlayStation 4 gamepad (DualShock 4). The DualShock 4 is readily available, has excellent operability and durability, and is also used for developing competition tasks.

.. note:: To use DualShock 3 or 4 with a PC, you need to prepare a separate Micro USB cable. For DualShock 4, you can also use a `USB Wireless Adapter <https://support.playstation.com/s/article/DUALSHOCK-4-USB-Wireless-Adapter?language=en_US>`_. Direct Bluetooth connection methods have not been confirmed.

Please connect the gamepad to the simulation PC beforehand.

.. _wrs_sample_controller_aizu_spider:

Aizu Spider Operation Method
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For Aizu Spider, the vehicle body is the main unit, with 0 to 2 JACO2 arms attached.

In gamepad operation, first switch the operation target with the logo button (PS button). Initially, the vehicle body is selected, and each button press cycles through:

・Vehicle body → Arm 1 → Arm 2 → Vehicle body → ...

The vehicle body operation methods are as follows:

.. list-table::
 :widths: 10, 10

 * - D-pad
   - Crawlers
 * - Left stick
   - Crawlers
 * - Right stick
   - All flipper elevation
 * - L2 button + Right stick
   - Left front flipper elevation
 * - R2 button + Right stick
   - Right front flipper elevation
 * - L1 button + Right stick
   - Left rear flipper elevation
 * - R1 button + Right stick
   - Right rear flipper elevation
 * - Right stick button (press right stick)
   - Align all flipper positions

| ※ The D-pad operates crawlers regardless of the selected operation target.
| ※ L1, L2, R1, R2 can be used in any combination. For example, pressing both L1 and L2 while operating the right stick allows simultaneous operation of the two left flippers.

The arm operation methods are as follows:

.. list-table::
 :widths: 10, 10

 * - Left stick horizontal axis
   - Arm joint 1
 * - Left stick vertical axis
   - Arm joint 2
 * - Right stick vertical axis
   - Arm joint 3
 * - Right stick horizontal axis
   - Arm joint 4
 * - X, B buttons (Square, Circle buttons)
   - Arm joint 5
 * - L1, R1 buttons
   - Arm joint 6
 * - L2, R2 triggers
   - Hand open/close

.. _wrs_sample_controller_doublearmv7:

Double-Arm Construction Robot Operation Method
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The gamepad operation methods for the double-arm construction robot (DoubleArmV7) are as follows:

.. list-table::
 :widths: 10, 10

 * - D-pad
   - Crawlers
 * - L1 button + Left stick
   - Crawlers
 * - L1 button + Right stick
   - Crawlers
 * - Left stick horizontal axis
   - Arm base yaw axis
 * - Right stick horizontal axis
   - Arm joint 1 yaw axis
 * - Left stick vertical axis
   - Arm joint 1 pitch axis
 * - Right stick vertical axis
   - Arm joint 2 pitch axis
 * - Y, A buttons (Triangle, X buttons)
   - End effector pitch axis
 * - X, B buttons (Square, Circle buttons)
   - End effector yaw axis
 * - L1 button + X, B buttons (Square, Circle buttons)
   - End effector roll axis
 * - R1 button, R2 trigger
   - End effector open/close
 * - Logo button (PS button)
   - Switch operation target arm

The L1 button switches the operation target for some sticks and buttons. For operations marked "L1 button + ...", operate the "..." part while holding the L1 button.

Supplementary Notes
^^^^^^^^^^^^^^^^^^^

The manual gamepad operation introduced here provides only the minimum necessary functions for testing and is very low-level, without even inverse kinematics operation for the end effector. In actual competitions, there's no need to use the same operation method. Rather than competing on manual operation skills, we hope competitors will demonstrate robot task execution capabilities through building efficient operation interfaces and automating robot behaviors.