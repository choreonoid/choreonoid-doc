==================================
Supplement: ODE Plugin Explanation
==================================

This section explains how to integrate a dynamics engine as a plugin for the core of simulation. The explanation uses ODEPlugin, which is a plugin implementation of the general dynamics engine ODE (Open Dynamics Engine). The source files are located in "src/ODEPlugin". For details about ODE, please refer to `http://www.ode.org/ <http://www.ode.org/>`_.

.. contents::
   :local:


ODE Plugin
----------

.. highlight:: cpp

First, here is the source code for the basic plugin structure. (The source code shown on this page is for explanatory purposes, and differs in some parts from the ODE plugin source code included in Choreonoid itself.) ::

 #include "ODESimulatorItem.h"
 #include <cnoid/Plugin>
 #include <ode/ode.h>
 #define PLUGIN_NAME "ODE"

 class ODEPlugin : public Plugin
 {
 public:
     ODEPlugin() : Plugin(PLUGIN_NAME)
     {
         require("Body");
     }
     virtual bool initialize()
     {
        ............
     }

     virtual bool finalize()
     {
        ...............
     }
 };
 CNOID_IMPLEMENT_PLUGIN_ENTRY(ODEPlugin);

Since it handles robot models, the constructor indicates that BodyPlugin is required. It also includes the ODE header file.

Plugin Initialization
~~~~~~~~~~~~~~~~~~~~~

This is the initialization function that is executed once after the plugin is loaded into memory. ::

 virtual bool initialize()
 {
     dInitODE2(0);
     dAllocateODEDataForThread(dAllocateMaskAll);

     ODESimulatorItem::initializeClass(this);
             
     return true;
 }

It calls ODE's initialization functions. (Functions with names starting with 'd' are ODE functions.) ODESimulatorItem will be explained in detail later. Here, we call its initialization function.

Plugin Finalization
~~~~~~~~~~~~~~~~~~~

This function is executed once when the plugin is destroyed (when Choreonoid exits). ::

 virtual bool finalize()
 {
     dCloseODE();
     return true;
 }

It calls ODE's termination function.
This completes the plugin structure.

ODE Simulator Item
------------------

The dynamics engine is implemented as a :ref:`simulation_simulator_item` so that Choreonoid can handle it as an item. In the ODE Plugin, ODESimulatorItem is implemented for using ODE in Choreonoid.

ODESimulatorItem is defined as a class that inherits from SimulatorItem. The header file that does this is shown below: ::

 #include <cnoid/SimulatorItem>
 #include "exportdecl.h"

 namespace cnoid {
         
 class CNOID_EXPORT ODESimulatorItem : public SimulatorItem
 {
 public:
     static void initializeClass(ExtensionManager* ext);
    ..........................
 };
 }

This is a static initialization function called during plugin initialization. It registers ODESimulatorItem with the itemManager that manages items, allowing ODESimulatorItem to be created from the menu. ::

 void ODESimulatorItem::initializeClass(ExtensionManager* ext)
 {
     ext->itemManager().registerClass<ODESimulatorItem>(ITEM_NAME);
     ext->itemManager().addCreationPanel<ODESimulatorItem>();
 }

When an ODE simulator item is added to the items, an object of the ODESimulatorItem class is created. In the constructor, initial values for user-modifiable parameters are set and variables are initialized. ::

 ODESimulatorItem::ODESimulatorItem()
 {
     initialize();
     stepMode.setSymbol(ODESimulatorItem::STEP_ITERATIVE,  N_("Iterative (quick step)"));
     gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
     .............
 }

The doDuplicate function is called when creating a new ODE simulator item. Please implement it to create a new object and return its pointer. ::

 ItemPtr ODESimulatorItem::doDuplicate() const
 {
     return new ODESimulatorItem(*this);
 }

When the ODE simulator item is deleted in the GUI, the object of the ODESimulatorItem class is also destroyed. In the destructor, release memory as needed. ::

 ODESimulatorItem::~ODESimulatorItem()
 {
     clear();
     if(contactJointGroupID){
         dJointGroupDestroy(contactJointGroupID);
     }
 }

This function is called when displaying parameters in the property view and when parameter values are changed.

.. code-block:: cpp

   void ODESimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
   {
       SimulatorItem::doPutProperties(putProperty);
       // Sets common properties for simulator items, so always call this.
    
       putProperty(_("Step mode"), stepMode, changeProperty(stepMode));
       // Function to set parameters. Specify the parameter name, variable, and function to call.
   }

Function to save parameter settings to the project file.

.. code-block:: cpp

   bool ODESimulatorItem::store(Archive& archive)
   {
       SimulatorItem::store(archive);
       // Saves common properties for simulator items, so always call this.
   
       archive.write("stepMode", stepMode.selectedSymbol());
       // Specify the name and variable of the parameter to save.
   
       write(archive, "gravity", gravity);
       // Use this function for Vector type variables.
   }

Function to read parameter settings from the project file.

.. code-block:: cpp

   bool ODESimulatorItem::restore(const Archive& archive)
   {
       SimulatorItem::restore(archive);
       // Reads common properties for simulator items, so always call this.

       archive.read("friction", friction);
       // Specify the name and variable of the parameter to read.

       read(archive, "gravity", gravity);
       // Use this function for Vector type variables.
   }

Simulation Implementation
~~~~~~~~~~~~~~~~~~~~~~~~~

Next is the implementation of the core simulation part. First, let's explain the overall flow.

When the user presses the simulation start button, the createSimulationBody function for creating ODE models is called as many times as there are models to be simulated.

Most dynamics engines have their own model description methods. ODE is no exception. In Choreonoid, robots and environments are held as Body objects. We need to construct ODE models from these Body objects.

The argument orgBody contains a pointer to the Body object, from which we create an ODEBody object for ODE and return its pointer. Here, we haven't created the actual ODE model yet. ::

 SimulationBodyPtr ODESimulatorItem::createSimulationBody(BodyPtr orgBody)
 {
     return new ODEBody(*orgBody);
 }

The ODEBody class is created by inheriting from the SimulationBody class. ::

 class ODEBody : public SimulationBody
 {
 public:
     ..................
 }
 
 ODEBody::ODEBody(const Body& orgBody)
     : SimulationBody(new Body(orgBody))
 {
    worldID = 0;
    ...............
 }

Next, the initialization function is called once. The argument simBodies contains pointers to the ODEBody objects created above for simulation.

.. code-block:: cpp

   bool ODESimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
   {
        clear();
        // Discard the results of the previous simulation.
   
        dRandSetSeed(0);
        dWorldSetGravity(worldID, g.x(), g.y(), g.z());
        dWorldSetERP(worldID, globalERP);
        .............
        // Set simulation parameters.

        timeStep = self->worldTimeStep();
        // Get the simulation time step with worldTimeStep().

        for(size_t i=0; i < simBodies.size(); ++i){
            addBody(static_cast<ODEBody*>(simBodies[i]));
        }
        // Build ODE models in the simulation world.
        // Call addBody for each target model to add models.

        return true;
    }

After that, the function to advance the simulation by one step is called repeatedly until the simulation ends. The argument activeSimBodies contains pointers to the ODEBody objects to be simulated.

.. code-block:: cpp
   
   bool ODESimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
   {
       for(size_t i=0; i < activeSimBodies.size(); ++i){
           ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);
           odeBody->body()->setVirtualJointForces();
           // Call BodyCustomizer function.

           odeBody->setTorqueToODE();
           // Set joint torques for each ODEBody object.
       }
   
       dJointGroupEmpty(contactJointGroupID);
       dSpaceCollide(spaceID, (void*)this, &nearCallback);
       // Perform collision detection.

       if(stepMode.is(ODESimulatorItem::STEP_ITERATIVE)){
           dWorldQuickStep(worldID, timeStep);
       } else {
           dWorldStep(worldID, timeStep);
       }
       // Advance the simulation time by one step.

       for(size_t i=0; i < activeSimBodies.size(); ++i){
           ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);

           if(!odeBody->sensorHelper.forceSensors().empty()){
               odeBody->updateForceSensors(flipYZ);
           }
           odeBody->getKinematicStateFromODE(flipYZ);
           if(odeBody->sensorHelper.hasGyroOrAccelSensors()){
               odeBody->sensorHelper.updateGyroAndAccelSensors();
           }
       }
       // Read the results after advancing one step from each ODEBody object.

       return true;
   }

.. note:: There is a description odeBody->body()->setVirtualJointForces() above. This is a mechanism called BodyCustomizer, which allows you to dynamically integrate model-specific programs into the dynamics calculation library. The sample project for this is CustomizedSpringModel.cnoid. The sample program is sample/SpringModel/SpringModelCustomizer.cpp. An explanation of this sample can be found on the OpenHRP3 homepage at `Joint Spring-Damper Modeling Method <http://www.openrtp.jp/openhrp3/en/springJoint.html>`_.


Body Class and Link Class
~~~~~~~~~~~~~~~~~~~~~~~~~

Before explaining the construction of ODE models, let's explain the Body class and Link class for describing physical objects in Choreonoid. (For information on VRML model description methods, please see `Robot and Environment Model Description Format <http://www.openrtp.jp/openhrp3/en/create_model.html>`_ on the OpenHRP3 homepage.)

Body objects manage Link objects that form a tree structure. Environmental models like floors are also Body objects consisting of a single Link object. A Body object always has a root link that is the root of the tree structure.

The Body class provides the following functions:

.. list-table:: Body Class Functions
   :widths: 30,60
   :header-rows: 1

   * - Function
     - Features
   * - int numJoints()
     - Returns the total number of joints.
   * - Link* joint(int id) 
     - Returns a pointer to the Link object corresponding to the joint id.
   * - int numLinks() 
     - Returns the total number of links.
   * - Link* link(int index)
     - Returns a pointer to the Link object corresponding to the link id.
   * - Link* link(const std::string& name)
     - Returns a pointer to the Link object matching the link name.
   * - Link* rootLink()
     - Returns a pointer to the root link.
   * - int numDevices()
     - Returns the total number of devices. The Device class is a parent class for describing force sensors, etc.
   * - Device* device(int index)
     - Returns the Device object corresponding to the device id.
   * - template<class DeviceType> DeviceList<DeviceType> devices()
     - | Returns a device list.
       | For example, to get a list of force sensor devices:
       | DeviceList<ForceSensor> forceSensors = body->devices();
   * - template<class DeviceType> DeviceType* findDevice(const std::string& name)
     - Returns a pointer to the Device object matching the device name.
   * - void initializeDeviceStates()
     - Initializes all devices to their initial state.
   * - bool isStaticModel()
     - Returns true for immovable objects like floors and walls.
   * - bool isFixedRootModel()
     - Returns true when the root link is a fixed joint.
   * - double mass()
     - Returns the total mass.
   * - const Vector3& centerOfMass() const;
     - Returns the center of mass vector
   * - void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false)
     - | Calculates forward kinematics (position and orientation of links other than the root link from the root link's position/orientation and all joint angles).
       | If calcVelocity and calcAcceleration are true, calculates link velocities and accelerations from joint angular velocities and accelerations.
   * - void clearExternalForces()
     - Sets external forces to zero.
   * - numExtraJoints()
     - Returns the number of virtual joints.
   * - ExtraJoint& extraJoint(int index)
     - Returns the virtual joint corresponding to the virtual joint id.


The Link class provides the following functions:

.. list-table:: Link Class Functions
   :widths: 30 60
   :header-rows: 1

   * - Function
     - Features
   * - Link* parent()
     - Returns a pointer to the parent link.
   * - Link* sibling()
     - Returns a pointer to the sibling link.
   * - Link* child()
     - Returns a pointer to the child link.
   * - bool isRoot()
     - Returns true if it is the root link.
   * - | Position& T()
       | Position& position()
     - Returns a reference to the position and orientation matrix of the link origin from world coordinates.
   * - Position::TranslationPart p()
     - Returns a reference to the position vector of the link origin from world coordinates.
   * - Position::LinearPart R()
     - Returns a reference to the link's orientation matrix from world coordinates.
   * - Position::ConstTranslationPart b()
     - Returns the position vector of the link origin from the parent link coordinate system.
   * - int jointId()
     - Returns the joint id.
   * - JointType jointType()
     - Returns the type of joint. There are rotational, translational, free, fixed, and (crawler).
   * - bool isFixedJoint()
     - Returns true for fixed joints.
   * - bool isFreeJoint()
     - Returns true for free joints.
   * - bool isRotationalJoint()
     - Returns true for rotational joints.
   * - bool isSlideJoint()
     - Returns true for translational joints.
   * - | const Vector3& a()
       | const Vector3& jointAxis()
     - Returns the rotation axis vector for rotational joints.
   * - const Vector3& d()
     - Returns the translation direction vector for translational joints.
   * - double& q()
     - Returns a reference to the joint angle.
   * - double& dq() 
     - Returns a reference to the joint angular velocity.
   * - double& ddq() 
     - Returns a reference to the joint angular acceleration.
   * - double& u() 
     - Returns a reference to the joint torque.
   * - const double& q_upper()
     - Returns a reference to the upper limit of joint movement.
   * - const double& q_lower() 
     - Returns a reference to the lower limit of joint movement.
   * - Vector3& v() 
     - Returns a reference to the velocity vector of the link origin from world coordinates.
   * - Vector3& w()
     - Returns a reference to the angular velocity vector of the link origin from world coordinates.
   * - Vector3& dv()
     - Returns a reference to the acceleration vector of the link origin from world coordinates.
   * - Vector3& dw()
     - Returns a reference to the angular acceleration vector of the link origin from world coordinates.
   * - | const Vector3& c()
       | const Vector3& centerOfMass()
     - Returns a reference to the center of mass vector from the link's own coordinate system.
   * - | const Vector3& wc() 
       | const Vector3& centerOfMassGlobal() 
     - Returns a reference to the center of mass vector from world coordinates.
   * - | double m() 
       | double mass() 
     - Returns the mass.
   * - const Matrix3& I()
     - Returns a reference to the inertia tensor matrix around the center of mass from the link's own coordinate system.
   * - const std::string& name()
     - Returns a reference to the link name.
   * - SgNode* shape()
     - Returns a pointer to the link's shape object.
   * - Matrix3 attitude() 
     - Returns the link's orientation matrix from world coordinates. (with offset)

.. note:: In Choreonoid, the local coordinate system representing the position and orientation of each link is set as follows: The coordinate origin is at the joint axis center. When all joint angles are 0 degrees, the orientation matrix is parallel to the world coordinate system. However, depending on the robot's structure, it may be more convenient to have an offset in the local coordinate system's orientation. In VRML file model descriptions, offset settings are possible. When Choreonoid reads a model file, even if an offset is set, it performs processing to change the local coordinate system as described above. The data obtained by the above functions is expressed in the coordinate system after the change. However, the orientation matrix obtained by the attitude() function is expressed in the coordinate system before the change.


ODE Model Construction
~~~~~~~~~~~~~~~~~~~~~~
Next, let's explain in detail about ODE model construction.

When the createSimulationBody function is called, we create an ODEBody object, but we're only preparing a container without any substance yet. The substance is created when addBody is called within initializeSimulation.

Here's the source code for addBody:

.. code-block:: cpp

   void ODESimulatorItemImpl::addBody(ODEBody* odeBody)
   {
        Body& body = *odeBody->body();
        // Get a pointer to the Body object.

        Link* rootLink = body.rootLink();
        // Get a pointer to the root link.
        rootLink->v().setZero();
        rootLink->dv().setZero();
        rootLink->w().setZero();
        rootLink->dw().setZero();
        // Set the root link's velocity, acceleration, angular velocity, and angular acceleration to zero.
   
        for(int i=0; i < body.numJoints(); ++i){
            Link* joint = body.joint(i);
            joint->u() = 0.0;
            joint->dq() = 0.0;
            joint->ddq() = 0.0;
        }
        // Also set each joint's torque, angular velocity, and angular acceleration to zero.
        // The root link's position, orientation, and each joint's angle are set to the simulation's initial values.
        
        body.clearExternalForces();
        // Set external forces to zero.
        body.calcForwardKinematics(true, true);
        // Calculate the position and orientation of each link.

        odeBody->createBody(this);
        // Create the ODE model.
    }

Here's the source code for createBody:

.. code-block:: cpp

   void ODEBody::createBody(ODESimulatorItemImpl* simImpl)
   {
       Body* body = this->body();
       // Get a pointer to the Body object.
   
       worldID = body->isStaticModel() ? 0 : simImpl->worldID;
       // Determine whether the model is an immovable object like a floor, and handle it differently.
   
       spaceID = dHashSpaceCreate(simImpl->spaceID);
       dSpaceSetCleanup(spaceID, 0);
       // ODE preparation.

       ODELink* rootLink = new ODELink(simImpl, this, 0, Vector3::Zero(), body->rootLink());
       // Create the model's root link (body).
       // Traverse from the root link to the end effectors to construct the whole.
       // Since the root link has no parent link, pass 0 for the parent link pointer and a zero vector for the position.

       setKinematicStateToODE(simImpl->flipYZ);
       // Set position and orientation to the ODEBody object.

       setExtraJoints(simImpl->flipYZ);
       // Set virtual joints.
      
       setTorqueToODE();
       // Set torques to the ODEBody object.

       sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);
       const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
       forceSensorFeedbacks.resize(forceSensors.size());
       for(size_t i=0; i < forceSensors.size(); ++i){
           dJointSetFeedback(
               odeLinks[forceSensors[i]->link()->index()]->jointID, &forceSensorFeedbacks[i]);
       }
       // Perform initial settings for sensor outputs like force sensors.
   
   }

Here's the source code for ODELink. It generates ODELink objects from Link object information.

.. code-block:: cpp

   ODELink::ODELink
   (ODESimulatorItemImpl* simImpl, ODEBody* odeBody, ODELink* parent,
    const Vector3& parentOrigin, Link* link)
   {
       ...................
   
       Vector3 o = parentOrigin + link->b();
       // Calculate the link origin position vector from world coordinates.
       // parentOrigin is the parent link's position vector.
   
       if(odeBody->worldID){
           createLinkBody(simImpl, odeBody->worldID, parent, o);
       }
       // Set physical data. In ODE, immovable objects don't need physical data, so we don't set it.
       
       createGeometry(odeBody);
       // Set shape data.
   
       for(Link* child = link->child(); child; child = child->sibling()){
           new ODELink(simImpl, odeBody, this, o, child);
       }
       // Traverse child links in order and create ODELinks.
   }

Here's the source code for createLinkBody that sets ODE physical data:

.. code-block:: cpp

   void ODELink::createLinkBody
   (ODESimulatorItemImpl* simImpl, dWorldID worldID, ODELink* parent, const Vector3& origin)
   {
       bodyID = dBodyCreate(worldID);
       // Create an ODE body (expressed as Body in ODE, corresponding to Link in Choreonoid).
   
       dMass mass;
       dMassSetZero(&mass);
       const Matrix3& I = link->I();
       dMassSetParameters(&mass, link->m(),
                          0.0, 0.0, 0.0,
                          I(0,0), I(1,1), I(2,2),
                          I(0,1), I(0,2), I(1,2));
       dBodySetMass(bodyID, &mass);
       // Set mass and inertia tensor matrix.

       ................
   
       dBodySetRotation(bodyID, identity);
       // Set the link's orientation.
       
       Vector3 p = o + c;
       dBodySetPosition(bodyID, p.x(), p.y(), p.z());
       // Set the link's position. In ODE, the center of mass is the link origin.

       dBodyID parentBodyID = parent ? parent->bodyID : 0;

       switch(link->jointType()){
       // Use different ODE joints depending on the joint type.
       
           case Link::ROTATIONAL_JOINT:
           // Use hinge joint for rotational joints.
           jointID = dJointCreateHinge(worldID, 0);
           dJointAttach(jointID, bodyID, parentBodyID);
           // Connect parent link and self link.
       
           dJointSetHingeAnchor(jointID, o.x(), o.y(), o.z());
           // The hinge joint position is at the Link object's origin.
       
           dJointSetHingeAxis(jointID, a.x(), a.y(), a.z());
           // Set the hinge joint's rotation axis.
           break;
       
           case Link::SLIDE_JOINT:
           // Use slider joint for translational joints.
           jointID = dJointCreateSlider(worldID, 0);
           dJointAttach(jointID, bodyID, parentBodyID);
           // Connect parent link and self link.
       
           dJointSetSliderAxis(jointID, d.x(), d.y(), d.z());
           // Set the slider joint's slide axis.
           break;

           case Link::FREE_JOINT:
           // For free joints, don't set anything.
           break;

           case Link::FIXED_JOINT:
           default:
           // For other or fixed joints
           if(parentBodyID){
               // If there's a parent link, connect to it with a fixed joint.
               jointID = dJointCreateFixed(worldID, 0);
               dJointAttach(jointID, bodyID, parentBodyID);
               dJointSetFixed(jointID);
               if(link->jointType() == Link::CRAWLER_JOINT){
                   simImpl->crawlerLinks.insert(make_pair(bodyID, link));
                   // Crawler joints are treated as fixed joints in ODE and handled as special cases in collision detection.
               }
           } else {
               dBodySetKinematic(bodyID);
               // If there's no parent link, set as KinematicBody (a body that doesn't move when collisions occur).
           }
           break;
       }
   }

Next is the source code for createGeometry that sets shape data. Shape data is described in a hierarchical structure within Shape objects.

.. code-block:: cpp
   
   void ODELink::createGeometry(ODEBody* odeBody)
   {
       if(link->shape()){
       // Get the Shape object.
       
           MeshExtractor* extractor = new MeshExtractor;
           // MeshExtractor is a utility class for traversing the hierarchy and extracting shape data.
           
           if(extractor->extract(link->shape(), [&](){ addMesh(extractor, odeBody); })){
           // Specify to call ODELink::addMesh each time a Mesh object is found while traversing the hierarchy.
           // When returning from the extract call, triangular mesh shapes are collected in vertices.
           
               if(!vertices.empty()){
                   triMeshDataID = dGeomTriMeshDataCreate();
                   dGeomTriMeshDataBuildSingle(triMeshDataID,
                                           &vertices[0], sizeof(Vertex), vertices.size(),
                                           &triangles[0],triangles.size() * 3, sizeof(Triangle));
                   // Convert to ODE data format.
                   
                   dGeomID gId = dCreateTriMesh(odeBody->spaceID, triMeshDataID, 0, 0, 0);
                   // Create ODE triangular mesh object.
                   geomID.push_back(gId);
                   dGeomSetBody(gId, bodyID);
                   // Associate with ODE Body.
               }
           }
           delete extractor;
       }
   }

In Choreonoid, when loading models, all shape data is converted to triangular mesh shapes, but if the original shape was a primitive type, that information is also preserved. The following code uses primitive types that ODE can handle as is, and creates types that cannot be handled as triangular mesh types.

Here's the source code for addMesh:

.. code-block:: cpp

   void ODELink::addMesh(MeshExtractor* extractor, ODEBody* odeBody)
   {
       SgMesh* mesh = extractor->currentMesh();
       // Get a pointer to the Mesh object.

       const Affine3& T = extractor->currentTransform();
       // Get the position and orientation matrix of the Mesh object.

       bool meshAdded = false;

       if(mesh->primitiveType() != SgMesh::MESH){
           // mesh->primitiveType() gets the shape data type.
           // There are MESH, BOX, SPHERE, CYLINDER, and CONE.
           // The following is processing when shape data is a primitive type.

           bool doAddPrimitive = false;
           Vector3 scale;
           optional<Vector3> translation;
           if(!extractor->isCurrentScaled()){
           // Returns true if there is a scale change.
               scale.setOnes();
               doAddPrimitive = true;
               // If there's no scale change, set scale vector elements to 1 and treat as primitive type.
           } else {
               // Processing when there is a scale change.

               Affine3 S = extractor->currentTransformWithoutScaling().inverse() *
                   extractor->currentTransform();
               // currentTransformWithoutScaling() gets the coordinate transformation matrix without scale transformation.
               // Extract only the scale transformation matrix.

               if(S.linear().isDiagonal()){
                   // Process only when the scale transformation matrix is diagonal.
                   // Otherwise, it cannot be handled as a primitive type in ODE.

                   if(!S.translation().isZero()){
                       translation = S.translation();
                       // If there's position translation in the scale matrix, save it.
                   }
                   scale = S.linear().diagonal();
                   // Assign diagonal elements to scale.

                   if(mesh->primitiveType() == SgMesh::BOX){
                       // If primitive type is Box, treat as primitive type.
                       doAddPrimitive = true;
                   } else if(mesh->primitiveType() == SgMesh::SPHERE){
                       if(scale.x() == scale.y() && scale.x() == scale.z()){
                           // If primitive type is Sphere and scale elements have the same value,
                           // treat as primitive type.
                           doAddPrimitive = true;
                       }
                       // Cannot treat as primitive type if scale elements don't have the same value.
                   } else if(mesh->primitiveType() == SgMesh::CYLINDER){
                       if(scale.x() == scale.z()){
                           // If primitive type is Cylinder and scale x,z elements have the same value,
                           // treat as primitive type.
                           doAddPrimitive = true;
                       }
                       // Cannot treat as primitive type if scale x,z elements don't have the same value.
                   }
               }
           }
           if(doAddPrimitive){
               // Processing when treating as primitive type. Create ODE primitive objects.

               bool created = false;
               dGeomID geomId;
               switch(mesh->primitiveType()){
               case SgMesh::BOX : {
                   const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                   // Get Box size.
                   geomId = dCreateBox(
                       odeBody->spaceID, s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
                   created = true;
                   break; }
               case SgMesh::SPHERE : {
                   SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                   // Get Sphere radius.
                   geomId = dCreateSphere(odeBody->spaceID, sphere.radius * scale.x());
                   created = true;
                   break; }
               case SgMesh::CYLINDER : {
                   SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                   // Get cylinder parameters.
                   geomId = dCreateCylinder(
                       odeBody->spaceID, cylinder.radius * scale.x(), cylinder.height * scale.y());
                   created = true;
                   break; }
               default :
                   break;
               }
               if(created){
                   geomID.push_back(geomId);
                   dGeomSetBody(geomId, bodyID);
                   // Associate ODE primitive object with ODE Body.
               
                   Affine3 T_ = extractor->currentTransformWithoutScaling();
                   // Get transformation matrix with scale removed.
               
                   if(translation){
                       T_ *= Translation3(*translation);
                       // Multiply by position translation included in scale matrix.
                   }
                   Vector3 p = T_.translation()-link->c();
                   // In ODE, link origin is center of mass, so correct for that.
               
                   dMatrix3 R = { T_(0,0), T_(0,1), T_(0,2), 0.0,
                                  T_(1,0), T_(1,1), T_(1,2), 0.0,
                                  T_(2,0), T_(2,1), T_(2,2), 0.0 };
                   if(bodyID){
                       dGeomSetOffsetPosition(geomId, p.x(), p.y(), p.z());
                       dGeomSetOffsetRotation(geomId, R);
                       // Set shape data position and orientation.
                   }else{
                       // For immovable objects, associate position/orientation matrix with id.
                       offsetMap.insert(OffsetMap::value_type(geomId,T_));
                   }
                   meshAdded = true;
               }
           }
       }

       if(!meshAdded){
           // Processing when not originally a primitive type or cannot be treated as a primitive type.

           const int vertexIndexTop = vertices.size();
           // Get the number of already added vertex coordinates.

           const SgVertexArray& vertices_ = *mesh->vertices();
           // Get reference to vertex coordinates in Mesh object.
       
           const int numVertices = vertices_.size();
           for(int i=0; i < numVertices; ++i){
               const Vector3 v = T * vertices_[i].cast<Position::Scalar>() - link->c();
               // Transform vertex vector coordinates.
               vertices.push_back(Vertex(v.x(), v.y(), v.z()));
               // Add to vertex coordinates vertices in ODELink object.
           }

           const int numTriangles = mesh->numTriangles();
           // Get total number of triangles in Mesh object.
           for(int i=0; i < numTriangles; ++i){
               SgMesh::TriangleRef src = mesh->triangle(i);
               // Get vertex numbers of the i-th triangle in Mesh object.
               Triangle tri;
               tri.indices[0] = vertexIndexTop + src[0];
               tri.indices[1] = vertexIndexTop + src[1];
               tri.indices[2] = vertexIndexTop + src[2];
               triangles.push_back(tri);
               // Add to triangle vertex numbers in ODELink object.
           }
       }
   }

This completes the construction of the ODE model.

Next, let's explain the functions that exchange data with the ODE model. Here's the source code for setKinematicStateToODE that sets the position, orientation, and velocity of ODE Body objects:

.. code-block:: cpp

   void ODELink::setKinematicStateToODE()
   {
       const Position& T = link->T();
       // Get the link's position and orientation matrix.
   
       if(bodyID){
           // Processing for moving objects.
       
           dMatrix3 R2 = { T(0,0), T(0,1), T(0,2), 0.0,
                           T(1,0), T(1,1), T(1,2), 0.0,
                           T(2,0), T(2,1), T(2,2), 0.0 };
   
           dBodySetRotation(bodyID, R2);
           // Set orientation matrix.
       
           const Vector3 lc = link->R() * link->c();
           const Vector3 c = link->p() + lc;
           // Convert link origin to center of mass.
       
           dBodySetPosition(bodyID, c.x(), c.y(), c.z());
           // Set position.
       
           const Vector3& w = link->w();
           const Vector3 v = link->v() + w.cross(lc);
           // Calculate velocity of link center of mass.
       
           dBodySetLinearVel(bodyID, v.x(), v.y(), v.z());
           dBodySetAngularVel(bodyID, w.x(), w.y(), w.z());
           // Set velocity and angular velocity.

       }else{
           // Processing for immovable objects. Update shape data position.
           for(vector<dGeomID>::iterator it = geomID.begin(); it!=geomID.end(); it++){
               OffsetMap::iterator it0 = offsetMap.find(*it);
               // For primitive types, position/orientation matrix from link local coordinates is mapped,
               // so multiply by that matrix.
               Position offset(Position::Identity());
               if(it0!=offsetMap.end())
                   offset = it0->second;
               Position T_ = T*offset;
               Vector3 p = T_.translation() + link->c();
               // Convert link origin to center of mass.
               
               dMatrix3 R2 = { T(0,0), T(0,1), T(0,2), 0.0,
                               T(1,0), T(1,1), T(1,2), 0.0,
                               T(2,0), T(2,1), T(2,2), 0.0 };

               dGeomSetPosition(*it, p.x(), p.y(), p.z());
               dGeomSetRotation(*it, R2);
               // Update shape data position and orientation information.
           }
       }
   }

Here's the source code for setTorqueToODE that sets torques to ODE Body objects:

.. code-block:: cpp

   void ODELink::setTorqueToODE()
   {
       if(link->isRotationalJoint()){
           // For rotational joints.
           dJointAddHingeTorque(jointID, link->u());
       } else if(link->isSlideJoint()){
           // For translational joints.
           dJointAddSliderForce(jointID, link->u());
       }
   }


Here's the source code for getKinematicStateFromODE that gets joint angles, angular velocities, link positions/orientations, and velocities from ODE Body objects:

.. code-block:: cpp

   void ODELink::getKinematicStateFromODE()
   {
       if(jointID){
           // Processing when there is a joint.
           if(link->isRotationalJoint()){
               // For rotational joints, get angle and angular velocity.
               link->q() = dJointGetHingeAngle(jointID);
               link->dq() = dJointGetHingeAngleRate(jointID);
           } else if(link->isSlideJoint()){
               // For slide joints, get position and velocity.
               link->q() = dJointGetSliderPosition(jointID);
               link->dq() = dJointGetSliderPositionRate(jointID);
           }
       }

       const dReal* R = dBodyGetRotation(bodyID);
       // Get ODE Body's orientation matrix.
   
       link->R() <<
           R[0], R[1], R[2],
           R[4], R[5], R[6],
           R[8], R[9], R[10];
       // Set to Link object's orientation matrix.
   
       typedef Eigen::Map<const Eigen::Matrix<dReal, 3, 1> > toVector3;
       const Vector3 c = link->R() * link->c();
       link->p() = toVector3(dBodyGetPosition(bodyID)) - c;
       // Get ODE Body position, convert from center of mass to joint position,
       // and set to Link object's position vector.
   
       link->w() = toVector3(dBodyGetAngularVel(bodyID));
       // Get ODE Body angular velocity and set to Link object's angular velocity vector.
   
       link->v() = toVector3(dBodyGetLinearVel(bodyID)) - link->w().cross(c);
       // Get ODE Body velocity, convert to joint position velocity,
       // and set to Link object's velocity vector.
   }

Collision Detection
-------------------

In the ODESimulatorItem::stepSimulation function, there is a line: ::

   dSpaceCollide(spaceID, (void*)this, &nearCallback);

This is an ODE function that searches for objects that might collide and calls the nearCallback function specified in the third argument. The second argument is used for parameter passing. In ODE, collision detection is performed this way, and constraint forces are generated between contacting objects within the nearCallback function. We'll omit detailed explanation about ODE here, but will explain the handling of crawler links.

Here's the source code for the nearCallback function:

.. code-block:: cpp

   static void nearCallback(void* data, dGeomID g1, dGeomID g2)
   {
       ...............

       ODESimulatorItemImpl* impl = (ODESimulatorItemImpl*)data;
       // Enable access to ODESimulatorItemImpl variables.

       ................
       if(numContacts > 0){
           // Processing when there is contact.
           dBodyID body1ID = dGeomGetBody(g1);
           dBodyID body2ID = dGeomGetBody(g2);
           Link* crawlerlink = 0;
           if(!impl->crawlerLinks.empty()){
               CrawlerLinkMap::iterator p = impl->crawlerLinks.find(body1ID);
               if(p != impl->crawlerLinks.end()){
                   crawlerlink = p->second;
               }
               // Check whether the contacted link is a crawler type.
               // (Currently, contact between crawler links is not assumed.)
               ..............................
           }
           for(int i=0; i < numContacts; ++i){
               dSurfaceParameters& surface = contacts[i].surface;
               if(!crawlerlink){
                   surface.mode = dContactApprox1;
                   surface.mu = impl->friction;
                   // For non-crawler links, set friction force.
               } else {
                   surface.mode = dContactFDir1 | dContactMotion1 | dContactMu2 | dContactApprox1_2;
                   // For crawler links, set surface velocity in friction direction 1 and friction force in friction direction 2.
                   const Vector3 axis = crawlerlink->R() * crawlerlink->a();
                   // Calculate crawler link's rotation axis vector.
                   const Vector3 n(contacts[i].geom.normal);
                   // Get contact point normal vector.
                   Vector3 dir = axis.cross(n);
                   if(dir.norm() < 1.0e-5){
                       // When these two vectors are parallel, set only friction force.
                       surface.mode = dContactApprox1;
                       surface.mu = impl->friction;
                   } else {
                       dir *= sign;
                       dir.normalize();
                       contacts[i].fdir1[0] = dir[0];
                       contacts[i].fdir1[1] = dir[1];
                       contacts[i].fdir1[2] = dir[2];
                       // Set the direction perpendicular to the two vectors as friction direction 1.
                       surface.motion1 = crawlerlink->u();
                       // Set surface velocity for friction direction 1.

               ............................

Sensor Output
-------------

Next, let's explain sensor output such as force sensors. Acceleration sensors, gyros, and force sensors attached to robots are described by the AccelSensor class, RateGyroSensor class, and ForceSensor class, respectively. BasicSensorSimulationHelper is a utility class that consolidates processing related to these sensors.

Here's the source code for sensor-related processing in the createBody function that builds models:

.. code-block:: cpp
   
   sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);
   // Initialize. The second argument is the simulation time step, the third is the gravity vector.
   
   // Then, make settings to get forces on joints from ODE.
   const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
   // Get the list of force sensor objects.
   forceSensorFeedbacks.resize(forceSensors.size());
   // Allocate storage area for the number of force sensors.
   for(size_t i=0; i < forceSensors.size(); ++i){
       dJointSetFeedback(
           odeLinks[forceSensors[i]->link()->index()]->jointID, &forceSensorFeedbacks[i]);
       // Sensor objects return the link object to which the sensor is attached with the link() function.
       // From that, get the ODE joint id. Specify the data storage location to ODE.
   }
   
In the stepSimulation function, perform the following processing:

.. code-block:: cpp
   
   for(size_t i=0; i < activeSimBodies.size(); ++i){
       ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);

       if(!odeBody->sensorHelper.forceSensors().empty()){
           odeBody->updateForceSensors(flipYZ);
           // If there are force sensors, call the updateForceSensors class.
       }
       
       odeBody->getKinematicStateFromODE(flipYZ);
       
       if(odeBody->sensorHelper.hasGyroOrAccelSensors()){
           odeBody->sensorHelper.updateGyroAndAccelSensors();
           // If there are gyro or acceleration sensors, call updateGyroAndAccelSensors().
           // In this function, sensor output values are calculated from Link object velocity and angular velocity.
       }
   }

Here's the source code for updateForceSensors:

.. code-block:: cpp
   
   void ODEBody::updateForceSensors(bool flipYZ)
   {
       const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
       // Get the list of force sensors.
   
       for(int i=0; i < forceSensors.size(); ++i){
           ForceSensor* sensor = forceSensors.get(i);
           const Link* link = sensor->link();
           // Get a pointer to the Link object to which the sensor is attached.
       
           const dJointFeedback& fb = forceSensorFeedbacks[i];
           Vector3 f, tau;
           f   << fb.f2[0], fb.f2[1], fb.f2[2];
           tau << fb.t2[0], fb.t2[1], fb.t2[2];
           // Get force and torque data on the joint from ODE.

           const Matrix3 R = link->R() * sensor->R_local();
           // R_local() function gets the sensor's orientation matrix from the link coordinate system to which the sensor is attached.
           // Multiply by the link's orientation matrix to convert to the sensor's orientation matrix from world coordinates.
           const Vector3 p = link->R() * sensor->p_local();
           // Similarly, p_local() function gets the sensor position.
           // Calculate the vector from link origin to sensor position in world coordinates.

           sensor->f()   = R.transpose() * f;
           // Convert to sensor coordinate system and assign to force data variable.
       
           sensor->tau() = R.transpose() * (tau - p.cross(f));
           // tau - p.cross(f) converts torque around link axis to torque around sensor position.
           // Further convert to sensor coordinate system and assign to torque data variable.
       
           sensor->notifyStateChange();
           // Function to emit a signal that sensor output has been updated.
       }
   }


What are Virtual Joints?
------------------------

When you set a virtual joint between two links, you can generate constraint forces between the specified links. This allows you to simulate closed-link mechanisms. A sample closed-link model is "share/model/misc/ClosedLinkSample.wrl".

This sample model has the following virtual joint definition: ::

 DEF J1J3 ExtraJoint {
     link1Name "J1"
     link2Name "J3"
     link1LocalPos 0.2 0 0
     link2LocalPos 0 0.1 0
     jointType "piston"
     jointAxis 0 0 1
 }

J1J3 is the name given to the virtual joint. link1Name and link2Name specify the names of the two links to constrain. link1LocalPos and link2LocalPos specify the constraint positions in each link's coordinate system. jointType specifies the constraint type. You can specify "piston" or "ball". jointAxis specifies the constraint axis as seen from link1's link coordinate system.

This information is stored in the Body object's ExtraJoint structure. The structure definition is: ::

 struct ExtraJoint {
         ExtraJointType type;
         Vector3 axis;
         Link* link[2];
         Vector3 point[2];
 };

It stores the values defined in the model file.

Next is the source code for setExtraJoint() that sets virtual joints in ODEBody objects:

.. code-block:: cpp

   void ODEBody::setExtraJoints(bool flipYZ)
   {
       Body* body = this->body();
       const int n = body->numExtraJoints();
       // Get the number of virtual joints.

       for(int j=0; j < n; ++j){
           Body::ExtraJoint& extraJoint = body->extraJoint(j);
           // Get reference to virtual joint.

           ODELinkPtr odeLinkPair[2];
           for(int i=0; i < 2; ++i){
               ODELinkPtr odeLink;
               Link* link = extraJoint.link[i];
               // Get pointer to link constrained by virtual joint.
           
               if(link->index() < odeLinks.size()){
                   odeLink = odeLinks[link->index()];               
                   if(odeLink->link == link){
                       odeLinkPair[i] = odeLink;
                       // Save the ODELink object corresponding to that Link object.
                   }
               }
               if(!odeLink){
                   break;
               }
           }

           if(odeLinkPair[1]){
               dJointID jointID = 0;
               Link* link = odeLinkPair[0]->link;
               Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
               // Convert Link1's constraint position to world coordinates.
           
               Vector3 a = link->attitude() * extraJoint.axis;
               // Convert constraint axis to world coordinates.
           
               if(extraJoint.type == Body::EJ_PISTON){
                   jointID = dJointCreatePiston(worldID, 0);
                   // Create piston joint.
                   dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                   // Connect two links with that joint.
                   dJointSetPistonAnchor(jointID, p.x(), p.y(), p.z());
                   // Specify joint position.
                   dJointSetPistonAxis(jointID, a.x(), a.y(), a.z());
                   // Specify joint axis.
               } else if(extraJoint.type == Body::EJ_BALL){
                   jointID = dJointCreateBall(worldID, 0);
                   // Create ball joint.
                   dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                   // Connect two links with that joint.
                   dJointSetBallAnchor(jointID, p.x(), p.y(), p.z());
                   // Specify joint position.
               }
           }
       }
   }
