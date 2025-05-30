.. highlight:: yaml

Additional Information
======================

Body files can describe arbitrary YAML mappings at the top level, and their contents can be retrieved from the program side.
The Choreonoid main body also has several features that utilize information described in this format.
Here we introduce the main ones among them.

Using this feature, it is also possible to perform :ref:`modelfile-yaml-add-information-to-another-model-format` for :doc:`modelfile-openhrp` and other formats.

.. note:: Regarding the keys in the descriptions explained below, the format explained in :ref:`body-file-reference-key-style` applies. Many keys were previously in camel case, so please note this when looking at old model files.

.. contents::
   :local:
   :depth: 1

Additional Information for SR1 Sample Model
-------------------------------------------

The following shows the additional information described in the SR1 sample model ("share/model/SR1/SR1.body"). Through this example, we will explain the specific description method for additional information. ::

 standard_pose: [ 
    0, -30, 0,  60, -30, 0,
   20, -10, 0, -40,   0, 0, 0,
    0, -30, 0,  60, -30, 0,
   20,  10, 0, -40,   0, 0, 0,
    0,   0, 0 
 ]
 
 link_group:
   - name: UPPER-BODY
     links:
       - WAIST_P
       - WAIST_R
       - CHEST
       - name: ARMS
         links:
           - name: R-ARM
             links: [ RARM_SHOULDER_P, RARM_SHOULDER_R, RARM_SHOULDER_Y, RARM_ELBOW, 
                      RARM_WRIST_Y, RARM_WRIST_P, RARM_WRIST_R ]
           - name: L-ARM
             links: [ LARM_SHOULDER_P, LARM_SHOULDER_R, LARM_SHOULDER_Y, LARM_ELBOW, 
                      LARM_WRIST_Y, LARM_WRIST_P, LARM_WRIST_R ]
   - WAIST
   - name: LEGS
     links:
       - name: R-LEG
         links: [ RLEG_HIP_R, RLEG_HIP_P, RLEG_HIP_Y, RLEG_KNEE, RLEG_ANKLE_P, RLEG_ANKLE_R ]
       - name: L-LEG
         links: [ LLEG_HIP_R, LLEG_HIP_P, LLEG_HIP_Y, LLEG_KNEE, LLEG_ANKLE_P, LLEG_ANKLE_R ]
 
 foot_links:
   - link: RLEG_ANKLE_R
     sole_center: [ 0.05, 0.0, -0.055 ]
   - link: LLEG_ANKLE_R
     sole_center: [ 0.05, 0.0, -0.055 ]
 
 default_ik_setup:
   WAIST: [ RLEG_ANKLE_R, LLEG_ANKLE_R ]
   RLEG_ANKLE_R: [ WAIST ]
   LLEG_ANKLE_R: [ WAIST ]
 
 collision_detection_rules:
   - disabled_link_chain_level: 3

Standard Pose Setting
---------------------

The "standard pose" introduced in :ref:`model_body_bar` of :doc:`../pose-editing` is described as additional information in the Body file. This is done in the following part. ::

 standard_pose: [ 
     0, -30, 0,  60, -30, 0,
    20, -10, 0, -40,   0, 0, 0,
     0, -30, 0,  60, -30, 0,
    20,  10, 0, -40,   0, 0, 0,
     0,   0, 0 
 ]

In this way, the joint angles corresponding to the standard pose are described as a list with the key "standard_pose". The order of joint angles is in joint ID order, and the unit of joint angles is [degree] ([m] for prismatic joints).

Link Group Structure Setting
----------------------------

In the "Link/Device View" introduced in :ref:`model_structure` of :doc:`../bodymodel`, a list of links that the model has is displayed, and the structure of the model can be confirmed. Also, links that are the target of editing operations can be selected here.

In this Link/Device View, the way the model structure is displayed can be switched with a combo box at the top, and among them there is a display method called "Grouped Tree". When this is selected, the SR1 model is displayed as follows.

.. image:: images/linkview_bodyparttree.png

Here, links are displayed hierarchically grouped by body parts. This makes it easier to understand the relationship between links and body parts. For this reason, this display method is also used in the key pose choreography function.

This hierarchical group structure is described in the following part starting with the key "link_group". ::

 link_group:
   - name: UPPER-BODY
     links:
       - WAIST_P
       - WAIST_R
       - CHEST
       - name: ARMS
         links:
           - name: R-ARM
             links: [ RARM_SHOULDER_P, RARM_SHOULDER_R, RARM_SHOULDER_Y,
                      RARM_ELBOW, 
                      RARM_WRIST_Y, RARM_WRIST_P, RARM_WRIST_R ]
           - name: L-ARM
             links: [ LARM_SHOULDER_P, LARM_SHOULDER_R, LARM_SHOULDER_Y, 
                      LARM_ELBOW, 
                      LARM_WRIST_Y, LARM_WRIST_P, LARM_WRIST_R ]
   - WAIST
   - name: LEGS
     links:
       - name: R-LEG
         links: [ RLEG_HIP_R, RLEG_HIP_P, RLEG_HIP_Y, 
                  RLEG_KNEE, 
                  RLEG_ANKLE_P, RLEG_ANKLE_R ]
       - name: L-LEG
         links: [ LLEG_HIP_R, LLEG_HIP_P, LLEG_HIP_Y,
                  LLEG_KNEE, LLEG_ANKLE_P,
                  LLEG_ANKLE_R ]


Here, groups and links classified into them are described using a combination of maps and lists. "name" represents the group name, and links belonging to that group or sub-groups are described under "links".

Foot Link Setting
-----------------

For legged models, by specifying which links are foot links and describing information about foot operations, you can utilize functions provided by Choreonoid for legged models. This is done in the following part. ::

 foot_links:
   - link: RLEG_ANKLE_R
     sole_center: [ 0.05, 0.0, -0.055 ]
   - link: LLEG_ANKLE_R
     sole_center: [ 0.05, 0.0, -0.055 ]

In this way, information about links corresponding to feet (having soles that can contact the floor) is listed with the key "foot_links". For each foot link information, the link name is described with the key "link", and the center point of the sole is described with the key "sole_center" in relative coordinates from the foot link. This enables functions such as :ref:`model_legged_body_bar`.

.. note:: The center point described in "sole_center" is intended to be the most stable point when the center of gravity projection point or ZMP is there, and does not necessarily need to be the geometric center. For example, if the ankle area is the stable point for control purposes, even if the ankle is connected at a position off-center from the sole, the ankle position should be set in sole_center.

.. _modelfile_yaml_preset_kinematics:

Preset Kinematics Setting
--------------------------

In the "preset kinematics mode" described in :doc:`../pose-editing` - :ref:`model_kinematics_mode`, forward kinematics and inverse kinematics are automatically switched according to the link the user is trying to move. This setting is done in the following part of the additional information file. ::

 default_ik_setup:
   WAIST: [ RLEG_ANKLE_R, LLEG_ANKLE_R ]
   RLEG_ANKLE_R: [ WAIST ]
   LLEG_ANKLE_R: [ WAIST ]

The settings made here are as follows:

* When moving the WAIST link (waist), perform inverse kinematics with both the RLEG_ANKLE_R link (right foot) and LLEG_ANKLE_R link (left foot) fixed as base links
* When moving the RLEG_ANKLE_R link, perform inverse kinematics with the WAIST link as the base link
* When moving the LLEG_ANKLE_R link, perform inverse kinematics with the WAIST link as the base link

In this way, you just need to specify the links you want to use inverse kinematics for in preset kinematics mode and their base links.

.. _modelfile_yaml_collision_detection:

Collision Detection Setting
---------------------------

Choreonoid can process collision detection between links. When collision detection is enabled in Choreonoid settings, basically all links become targets for collision detection. (However, collision detection can be enabled or disabled for each body through the properties of the body item. Self-collision detection can also be switched.)

For joints embedded in other links or joints that combine multiple rotation axes, collisions inside the joint should originally be designed not to occur within the movable range, but it can be time-consuming to create model file shapes in such detail. Conversely, for links covered with flexible surfaces, collisions may be allowed by design. In such cases, by excluding specific links or specific link pairs from collision detection targets, it becomes possible to appropriately handle collision detection in Choreonoid.

This setting can be described in "collision_detection_rules". In SR1, it is described as follows. ::

 collision_detection_rules:
   - disabled_link_chain_level: 3

As elements directly under collision_detection_rules, YAML lists are described, and rules are described for each element of the list. This allows multiple rules to be combined.

Regarding "disabled_link_chain_level", this is a setting that excludes links adjacent in parent-child relationships in the joint tree from self-collision. If this rule is not described or the value is set to 0, it checks for collisions among all link pairs included in the target body. On the other hand, if a value of 1 or more is set here, pairs whose distance between nodes in the link tree is less than or equal to that value are excluded from self-collision targets. For example, if 1 is set, links in direct parent-child relationships are excluded from self-collision checks, and if 2 is set, links that are grandparents, grandchildren, or siblings of a certain link are also excluded.

The available rules are shown in the table below.

.. list-table:: Collision Detection Rules
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - Rule (Key)
   - Description
 * - disabled_link_chain_level
   - Distance in the link tree to disable collision detection for self-collision of the target body
 * - disabled_links
   - Describe links to disable collision detection in list format
 * - disabled_link_group
   - Describe groups of links to disable collision detection in list format. Collision detection between links described here will not be performed.
 * - enabled_links
   - Describe groups of links to enable collision detection in list format.
 * - enabled_link_group
   - Describe groups of links to enable collision detection in list format. Collision detection between links described here will be performed.

As a basic rule, collision detection is enabled for all links by default.
When collision detection is enabled for a link, collision detection is performed with all other links that have collision detection enabled.

In contrast, by describing rules starting with disabled\_, you can set links to be excluded from collision detection targets.
However, you may want to re-enable collision detection for some of the links that were excluded collectively by disabled_link_chain_level or disabled_link_group. In that case, you can additionally describe rules starting with enabled\_.
When there are multiple such rules, they are applied in the order they are described.
Therefore, the general description method is to first describe rules for disabling, and then describe rules for re-enabling part of them as needed.

Other Information Description
-----------------------------

Above, we explained the main information described in the SR1 sample, but additional information can describe any information as long as it follows YAML syntax and does not conflict with existing keys in the model file. The content can be read inside Choreonoid, and each function can obtain necessary information from this. By describing information required by newly introduced plugins, the plugin functions become available, and even when users develop plugins, users can define and use necessary information. In this way, additional information using YAML can be handled flexibly and serve as an important mechanism for extending Choreonoid functionality.

.. _modelfile-yaml-add-information-to-another-model-format:

Adding Information to Other Format Model Files
-----------------------------------------------

In SR1.body, the model file is described in Choreonoid's standard Body format, and the above additional information is also described together within that file.

However, when you want to use existing model files described in other formats as they are, you may want to set additional information for model files other than Body format.

In that case, first prepare a YAML file that describes the additional information. The extension is usually set to .yaml.

Then describe the additional information there.

On top of that, make the following description in the YAML file. ::

 model_file: model_file_name

For example, if there is a model file "robot.wrl" described in OpenHRP format, ::

 model_file: robot.wrl

and so on.

If the YAML file describing additional information and the main model file are in the same directory, only the filename of the main file is OK. If they are in different directories, describe with the relative path or absolute path to that directory.

Then, when loading from Choreonoid, load the YAML file.

By doing this, the main model information is loaded in other formats, while additional information for that model is also loaded.