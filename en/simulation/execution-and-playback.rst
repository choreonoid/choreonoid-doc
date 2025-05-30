Simulation Execution and Playback
=================================

.. sectionauthor:: Shin'ichiro Nakaoka <s.nakaoka@aist.go.jp>

.. contents:: Table of Contents
   :local:

.. _simulation_start_simulation:

Starting a Simulation
---------------------

To start a simulation, perform the following operations after completing :doc:`simulation-project`:

1. Select the target simulator item in the item tree view
2. Press the simulation start button

Operation 1 is not necessary if only one simulator item exists. In this case, since there is only one simulation to execute, no operation is needed to identify it.

On the other hand, a project can contain multiple simulator items. This is useful when you want to prepare multiple simulation items with different parameters, run simulations for each, and compare the results. In such cases, you need to select a simulator item to identify which simulation to execute.

.. note:: Remember that identifying the target simulator item applies to pause and stop operations as well, which are explained below. In any case, if only one simulator item exists, you don't need to worry about this.

For operation 2, use the buttons on the simulation bar. Normally, use the following "Start simulation" button to start a simulation:

.. image:: ../basics/images/SimulationBar_StartButton.png

When you start a simulation with this button, it begins from the position and orientation of the initial state registered in :ref:`simulation_setting_initial_status`.

You can also use the following "Resume simulation" button to start a simulation:

.. image:: images/simbar-restart-button.png

In this case, unlike with the start button, the simulation starts from the current model state rather than the registered initial state.

Pausing a Simulation
--------------------

You can pause a running simulation by pressing the following pause button:

.. image:: images/simbar-pause-button.png

Press this button again or the "Resume simulation" button mentioned earlier to resume the paused simulation.


Stopping a Simulation
---------------------

Press the following "Stop simulation" button to stop a simulation:

.. image:: images/simbar-stop-button.png

In this case, unlike pausing, all elements involved in the simulation (including controllers) are terminated (as much as possible). Therefore, you cannot resume from exactly the same state as when it was stopped. (You can start a simulation from the position and orientation at the time of stopping using the "Resume simulation" button, but this only sets those as the initial position and orientation, and the simulation restarts with all other elements initialized.)

On the other hand, resources used for simulation execution are released, and it becomes possible to restart the simulation from an initialized state. Therefore, use the stop operation when you don't need to resume the simulation.


.. _simulation-time-range:

Setting Time Range
------------------

You can configure when to end a simulation using the simulator item's "Time range" property. Select from the following options:

* **Unlimited**

 Does not specify an end time. The simulation continues until an explicit stop operation is performed. This is the default setting.

* **Active control period**

 The simulation continues while controllers are actively controlling. The simulation ends when all controllers finish their control.

* **Specified time**

 The simulation runs for the time specified in the "Time length" property.

* **Time bar range**

 The simulation runs for the time bar range.

※ The time specified here is the time in the virtual world within the simulator.

When the time range is set to anything other than "Unlimited", the simulation automatically ends at the corresponding timing. In this case, no explicit stop operation is necessary. Of course, you can still stop the simulation midway with an explicit stop operation.

When the time range is set to "Unlimited", the simulation continues until an explicit stop operation is performed.

.. _simulation-realtime-sync:

Synchronization with Real Time
------------------------------

Generally, the progression of time in the virtual world where simulation occurs differs from that in the real world. The progression of time in the virtual world depends on the simulation's computation time and can be faster or slower than the real world.

However, there are cases where you want to match the virtual world's time progression to real time. One reason is that it's easier to grasp the simulation's progress. It's also more suitable when you want interaction between the simulation and the real world. For example, when a real-world operator controls a robot during simulation, having synchronized time progression provides a sensation closer to operating an actual robot.

In Choreonoid, matching the time progression between the virtual and real worlds is called "real-time synchronization". This can be enabled with the simulator item's "Sync to actual time" property. The default is True (enabled). It's usually best to run simulations with this setting for clarity.

However, note that real-time synchronization is only effective when computation time is faster than real time. When computation time is slower than real time, the simulation's time progression doesn't change regardless of the synchronization setting. It's impossible to make slow computation time faster.

Conversely, when computation time is faster than real time, disabling real-time synchronization allows the simulation to progress faster than real time. Try this setting when you want to reduce the time spent on simulation.

.. _simulation-result-recording:

Recording Simulation Results
----------------------------

Simulation results can be recorded as time-series data for use in result playback and analysis.

This feature is configured using the simulator item's "Recording mode" property. Select from the following modes:

* **Full**

 Records all results from start to finish of the simulation.

* **Tail**

 Records a certain period before the simulation ends. Older portions beyond this period are discarded. The period is set with the "Time length" property.

* **Off**

 No recording is performed. Simulation results can only be viewed during simulation execution.

The default mode is "Full". Since it's best to be able to replay and analyze simulation results for the entire period, normally select this recording mode.

However, recording simulation results requires memory space. As the number of simulated objects increases or simulation time lengthens, memory usage increases accordingly. If memory used for recording simulation results exceeds available capacity, Choreonoid may crash during simulation, so caution is needed.

The "Tail" mode is provided to avoid such situations. This mode keeps memory usage within a certain range by discarding temporally old portions when the specified time length is exceeded. If you set a time length considering system memory capacity, the system won't crash due to memory shortage even during long simulations. Therefore, use this feature when:

* Running long simulations
* System memory capacity is insufficient
* Recording the entire period isn't essential, but you want to keep recent records for problem analysis

.. note:: When long simulations and their complete recording are necessary, you can also use "World log file items" to continuously write results to files. In this case, as long as there's sufficient free space in the file system, insufficient memory capacity isn't a problem.

.. For details, see hogehoge.

When recording mode is "Off", no results are recorded. Even in this case, you can check simulation progress as results are continuously reflected in the model. However, you won't be able to replay or analyze results later.

.. _simulation-device-state-recording:

Recording Device States
-----------------------

The basic elements recorded as simulation results are the motion trajectory data needed to replay the model's physical movements. Additionally, changes in device states can also be recorded. To do this, run the simulation with the simulator item's "Record device states" property set to true. This enables replay of sensor state changes and device operations like on/off switching. However, this increases both the required memory and processing overhead, so toggle this feature as needed.

.. _simulation-result-playback:

Playing Back Simulation Results
-------------------------------

When simulation result recording is enabled, you can play them back as animations. To play back results:

1. Select the target simulator item in the item tree view
2. Operate the time bar

For time bar operation, pressing the play button displays animation at a constant speed, and operating the time slider allows playback of any desired portion. See :doc:`../basics/timebar` for details.


.. _simulation_playback_ongoing_simulation:

Displaying Ongoing Simulations
------------------------------

When simulation result recording is enabled, ongoing simulation display also uses the playback function described above as "playback of data being recorded". However, there are some behavioral differences between ongoing and completed simulation playback. Specifically:

1. No additional playback operations are needed when starting a simulation. Animation of the result display starts without needing to reselect the simulator item or operate the time bar.

2. During simulation, animation continues even if the simulator item is deselected. This continues until another simulator item is selected or an animation stop operation is performed.

3. You can resume playback of an ongoing simulation by pressing the "Start simulation" or "Resume simulation" buttons on the simulation bar.

4. When "Sync with ongoing updates" is on in the time bar settings, the latest state is always displayed during simulation.

Note that the simulation itself continues even if animation is stopped. You can restore the ongoing simulation display by performing operation 4. When restoring playback through normal operations, the setting in 4 is involved, so please be aware of this.

.. note:: When "Sync with ongoing updates" is off, the simulation's internal progress and the animation progress displaying results don't necessarily match. In that case, if you perform interactive operations on the ongoing simulation, responses may not return immediately, so caution is needed. This isn't a problem if "Sync with ongoing updates" is on. It's on by default.

.. _simulation-result-item-output:

Output Destination of Simulation Results
----------------------------------------

Each model's motion trajectory is output as a child item of the corresponding body item with the name "Simulator item name - Model name".

For example, in the project created in :doc:`simulation-project`, after starting the simulation, an "AISTSimulator-box1" item is output under the box1 model as follows: ::

 [ ] - World
 [/]   + box1
 [ ]     + AISTSimulator-box1
 [/]   + Floor
 [ ]   + AISTSimulator

.. images/simproject-item4.png

Here, since the "Floor" model is a static model, no motion trajectory is output.

The output item type is "Body motion item" introduced in :ref:`basics_sequence_data`. Body motion items are defined as composite items with the following sub-item structure:

| + AISTSimulator-box1
|   + Joint
|   + Cartesian

Joint stores joint angle trajectories, and Cartesian stores link position/orientation trajectories. These are MultiValueSeq items and MultiSE3Seq items, respectively.

Furthermore, if device states are also recorded, that data is output to "Devices" as follows:

| + AISTSimulator-box1
|   + Joint
|   + Cartesian
|   + Devices

The Devices type is MultiDeviceStateSeq item.

Data output this way is simply project items with their designated types. Therefore, any operations valid for each item type can be used for simulation results as well. As specific examples, you can save trajectory data from items to files and reload them later, or visualize trajectories in the graph view.

Motion trajectory playback can also be performed by selecting these items and operating the time bar. However, in that case, only the selected item's trajectory is played back. If a simulation has multiple dynamic models, you need to select all models' motion trajectory items to replay the entire simulation. However, you don't actually need to do this much - as explained earlier, selecting the corresponding simulator item makes the entire simulation the playback target.