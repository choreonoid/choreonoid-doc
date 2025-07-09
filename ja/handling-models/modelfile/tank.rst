
SimpleTankモデルファイル全記述内容
==================================

.. highlight:: YAML
   :linenothreshold: 5

:doc:`modelfile-newformat` で解説したTankモデルを記述しているモデルファイルの全テキストを以下に掲載します。本モデルは Choreonoid インストール先の "share/model/tank/SimpleTank.body" というファイルに格納されています。 ::

 format: ChoreonoidBody
 format_version: 2.0
 name: SimpleTank
 
 links:
   -
     name: CHASSIS
     translation: [ 0, 0, 0.1 ]
     joint_type: free
     center_of_mass: [ 0, 0, 0 ]
     mass: 8.0
     inertia: [
       0.1, 0,   0,
       0,   0.1, 0,
       0,   0,   0.5 ]
     elements:
       Shape:
         geometry:
           type: Box
           size: [ 0.45, 0.3, 0.1 ]
         appearance: &BodyAppearance
           material:
             diffuse: [ 0, 0.6, 0 ]
             specular: [ 0.2, 0.8, 0.2 ]
             specular_exponent: 80
   -
     name: TURRET_Y
     parent: CHASSIS
     translation: [ -0.04, 0, 0.1 ]
     joint_type: revolute
     joint_axis: -Z
     joint_range: unlimited
     max_joint_velocity: 90
     joint_id: 0
     center_of_mass: [ 0, 0, 0.025 ]
     mass: 4.0
     inertia: [
       0.1, 0,   0,
       0,   0.1, 0,
       0,   0,   0.1 ]
     elements:
       Shape:
         geometry:
           type: Box
           size: [ 0.2, 0.2, 0.1 ]
         appearance: *BodyAppearance
   -
     name: TURRET_P
     parent: TURRET_Y
     translation: [ 0, 0, 0.05 ]
     joint_type: revolute
     joint_axis: -Y
     joint_range: [ -10, 45 ]
     max_joint_velocity: 90
     joint_id: 1
     elements:
       - 
         # Turret
         type: RigidBody
         center_of_mass: [ 0, 0, 0 ]
         mass: 3.0
         inertia: [
           0.1, 0,   0,
           0,   0.1, 0,
           0,   0,   0.1 ]
         elements:
           Shape:
             geometry:
               type: Cylinder
               height: 0.1
               radius: 0.1
             appearance: *BodyAppearance
       - 
         # Gun
         type: Transform
         translation: [ 0.2, 0, 0 ]
         rotation: [ 0, 0, 1, 90 ]
         elements:
           RigidBody:
             center_of_mass: [ 0, 0, 0 ]
             mass: 1.0
             inertia: [
               0.01, 0,   0,
               0,    0.1, 0,
               0,    0,   0.1 ]
             elements:
               Shape:
                 geometry:
                   type: Cylinder
                   height: 0.2
                   radius: 0.02
                 appearance: *BodyAppearance
       -
         type: SpotLight
         name: Light
         translation: [ 0.08, 0, 0.1 ]
         direction: [ 1, 0, 0 ]
         beam_width: 36
         cut_off_angle: 40
         cut_off_exponent: 6
         attenuation: [ 1, 0, 0.01 ]
         elements:
           Shape:
             rotation: [ 0, 0, 1, 90 ]
             translation: [ -0.02, 0, 0 ]
             geometry:
               type: Cone
               height: 0.04
               radius: 0.025
             appearance:
               material:
                 diffuse: [ 1.0, 1.0, 0.4 ]
                 ambient: 0.3
                 emissive: [ 0.8, 0.8, 0.3 ]
       - 
         type: Camera
         name: Camera
         translation: [ 0.1, 0, 0.05 ]
         rotation: [ [ 1, 0, 0, 90 ], [ 0, 1, 0, -90 ] ]
         format: COLOR_DEPTH
         field_of_view: 65
         width: 320
         height: 240
         frame_rate: 30
         elements:
           Shape:
             rotation: [ 1, 0, 0, 90 ]
             geometry:
               type: Cylinder
               radius: 0.02
               height: 0.02
             appearance:
               material:
                 diffuse: [ 0.2, 0.2, 0.8 ]
                 specular: [ 0.6, 0.6, 1.0 ]
                 specular_exponent: 80
   -
     name: TRACK_L
     parent: CHASSIS
     translation: [ 0, 0.2, 0 ]
     joint_type: pseudo_continuous_track
     joint_axis: Y
     centerOfMass: [ 0, 0, 0 ]
     mass: 1.0
     inertia: [
       0.02, 0,    0,
       0,    0.02, 0,
       0,    0,    0.02 ]
     elements:
       Shape: &TRACK 
         geometry:
           type: Extrusion
           cross_section: [
             -0.22, -0.1,
              0.22, -0.1,
              0.34,  0.06,
             -0.34,  0.06,
             -0.22, -0.1
             ]
           spine: [ 0, -0.05, 0, 0, 0.05, 0 ]
         appearance:
           material:
             diffuse: [ 0.2, 0.2, 0.2 ]
   -
     name: TRACK_R
     parent: CHASSIS
     translation: [ 0, -0.2, 0 ]
     joint_type: pseudo_continuous_track
     joint_axis: Y
     centerOfMass: [ 0, 0, 0 ]
     mass: 1.0
     inertia: [
       0.02, 0,    0,
       0,    0.02, 0,
       0,    0,    0.02 ]
     elements:
       Shape: *TRACK 
