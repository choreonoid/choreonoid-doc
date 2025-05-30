============================
Creating a Project Item Type
============================

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

Overview
--------

For project items, which are one of the main components in the Choreonoid framework, it is also possible to add custom item types through plugins. This allows you to freely implement functionality that cannot be achieved with existing item types alone.

This section explains the basics of creating custom item types.
In subsequent sections, we will introduce specific examples along with samples.

Flow of Creating an Item Type
-----------------------------

A custom item type is first defined as a class that inherits from the `Item class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Item.html>`_. For that class, implement the actual content of the item as shown below:

* Hold/implement objects and data handled by the custom item
* Define/implement functions to manipulate objects and data
* Define/emit signals for notifications from objects and data
* Customize behavior as an item

"Customizing behavior as an item" can be achieved by the following means:

* Override various virtual functions defined in the Item class
* Inherit and implement interface classes for specific functionality

Customizable items include:

* Behavior when operations such as add, rename, move, delete are performed on the item tree
* Saving to and loading from projects
* Display and editing of properties
* Drawing and interaction on the scene view
* Coordination with other fundamental views

Finally, register the created item class with ItemManager.
This makes it available as a project item in Choreonoid.
At this time, you can also customize new creation from menus and file input/output.

Also, for items of the created type, you may not be able to achieve the necessary display, operation, or editing with existing interfaces.
In that case, you can address this by creating custom toolbars or views. We have already explained :doc:`toolbar`, and custom views can also be added through plugins. This will be explained later in this guide.

Below we explain an overview of each of the above items.

Defining a Custom Item Type
---------------------------

Define a custom item type by inheriting from the Item class as follows: ::

 class FooItem : public cnoid::Item
 {
 public:
     FooItem();
     FooItem(const FooItem& org);
     ~FooItem();
 
     virtual Item* doDuplicate() const override;
 
     ...
 };

The class name should end with "Item".
Define constructors and destructors as needed.
Basically, it's OK if it can be generated with a default constructor.

Items need to override the doDuplicate() function to be able to create duplicates of themselves. This is usually achieved by implementing: ::

 Item* FooItem::doDuplicate() const
 {
     return new FooItem(*this);
 }

Since item duplication is a function required within the Choreonoid framework, you need to implement it even if you don't perform duplication in your own code.

.. note:: Items can also be created as singletons. In that case, you don't need to be able to duplicate them.

Holding/Implementing Objects and Data
-------------------------------------

For an item to be meaningful, it needs to have some functionality or data.
There are two main ways to implement these in an item class:

1. Hold other objects or data
2. Implement directly in the item class

As an example of 1, for BodyItem which corresponds to robot and environment models, there is first the `Body class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Body.html>`_ that can be used independently of items, and the `BodyItem class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1BodyItem.html>`_ is defined to hold a pointer to that object. Here, the BodyItem class functions as a wrapper class to handle Body objects in Choreonoid's GUI.

As can be understood from such examples, custom item types don't necessarily need to implement all their functionality from scratch. Rather, you can think of the item class as something you additionally implement when there are existing classes or data that you want to handle in Choreonoid. In that case, you can realize most of the item's functionality by first having the item class hold the corresponding objects or data in the form of member variables or pointers, and making them accessible from outside.

Alternatively, even when creating a custom item type from scratch including its functionality and data parts, you might consider deliberately defining those essential parts separately from the item class, allowing them to be used independently of Choreonoid's GUI. In the example of BodyItem and Body classes, you could think of it as deliberately separating the GUI-independent parts into the Body class, even though everything could have been implemented as BodyItem from the beginning. And such GUI-independent parts are organized into an independent library called the Body library, which is separated as a module from the Body plugin.

Keep this in mind when implementing custom items for more flexible handling.
Of course, depending on the type of item, the form 2 of implementing everything together in the item class may be better, so please make appropriate choices according to the situation.

Defining/Emitting Signals
-------------------------

We introduced :doc:`signals` as an important concept in the Choreonoid SDK. This can also be utilized in implementing custom items. For example, if you define a signal that notifies which part of the data was updated and how when data held by an item is updated, and emit this signal when updating data, it becomes easier to implement functionality that coordinates with that data. By properly using the characteristic of signals to coordinate with loose coupling, you can make the implementation concise and improve extensibility and maintainability.

In fact, each existing item type in Choreonoid has its own signals defined and utilized. When creating custom item types, it would be good to utilize signals as needed. When defining signals in an item class that can be used from outside, they are usually made accessible through member functions that return :ref:`plugin-dev-signal-proxy` objects.

Overriding Virtual Functions
----------------------------

Besides the doDuplicate function mentioned above, the Item class has several virtual functions that are intended to be overridden.
By actually overriding these functions, you can customize the basic behavior as an item.
Below we introduce the relevant virtual functions by category.

Name-related
~~~~~~~~~~~~

* **virtual bool setName(const std::string& name)**

  Executed when the item's name is set in Choreonoid's GUI. If the object held by the item class needs to have a name set, update the name in this function.

.. _plugin-dev-item-virtual-function-on-item-tree:

Item Tree-related
~~~~~~~~~~~~~~~~~

There are several virtual functions executed in response to changes in the item tree. Here we introduce the main ones.

* **virtual void onConnectedToRoot()**

  Executed when directly or indirectly connected to the root item. Since items become operable in the GUI when connected to the root item, you can ensure initialization at the necessary timing by implementing initialization processing here.

* **virtual void onTreePathChanged()**

  Executed when the path from root to item in the item tree changes. Same conditions as the Item class signal sigTreePathChanged. (The signal is emitted after this function is executed.) When determining the item's behavior from relative positional relationships with other items in the tree, implementing initialization processing in this function ensures reliable relationship updates.

* **virtual void onTreePositionChanged()**

  Executed when the item's position in the item tree changes. This is executed when onTreePathChanged is executed, but also when the order among sibling items changes even if the path remains the same. Same conditions as the Item class signal sigTreePositionChanged. (The signal is emitted after this function is executed.)
  
* **virtual void onDisconnectedFromRoot()**

  Executed when the connection with the root item is released. Contrary to connection time, implementing item termination processing such as releasing used resources here allows execution at appropriate timing.
  

Project Save/Restore-related
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following functions are defined to make items compatible with project saving:

* **virtual bool store(Archive& archive)**

  This function is executed for each item during project saving. By implementing processing to save the item's state and data here, you can realize project saving for items. The argument archive is an object that stores structured data, and you write item information to it.
  
* **virtual bool restore(const Archive& archive)**

  This function is executed for each item during project loading. By implementing processing to restore the item's state and data here, you can make items compatible with project loading. The argument archive contains the information saved, so you extract information from it to restore the item's state and data.

Details on how to implement these functions will be explained separately.  

Property-related
~~~~~~~~~~~~~~~~

Items have a concept called "properties", which can be displayed and edited on the item property view, a basic feature of Choreonoid. The following is a function to make items compatible with this feature:

* **virtual void doPutProperties(PutPropertyFunction& putProperty)**

  Outputs the item's properties. The argument putProperty is an object with property output functionality, and you output properties through it. Values edited there can be obtained through callback functions passed to putProperty, allowing you to implement property update processing.

Details on how to implement this function will be explained separately.  

Introducing Interface Classes for Fundamental View Coordination
---------------------------------------------------------------

Items can coordinate with several fundamental views that Choreonoid provides.
To do this, inherit interface classes for coordination and override their virtual functions.
The following interface classes are available:

* **RenderableItem**

  Interface for coordinating items with the scene view. If an item is a drawable object, introducing this allows the item to be displayed and edited on the scene view.

* **LocatableItem**

  Interface for coordinating items with the Location view. If an item is an object that can be placed in three-dimensional virtual space, introducing this allows the object's position and orientation to be displayed and edited on the location view.

* **ImagableItem**

  Interface for coordinating items with the Image view. If an item contains two-dimensional image data, introducing this allows the item to be displayed on the image view.

Here we briefly introduce the definition and usage of RenderableItem.
The main parts of this interface class can be represented as follows: ::

 class RenderableItem
 {
 public:
     virtual SgNode* getScene() = 0;
 };

Here, SgNode is a class corresponding to nodes in a three-dimensional scene graph, and these objects can be drawn on the scene view.

An item class that introduces this is defined as follows: ::

 class FooItem : public cnoid::Item, public RenderableItem
 {
 public:
     ...

     virtual SgNode* getScene() override;

     ...
 };

It also inherits RenderableItem and overrides the getScene function. If you make this getScene function return the SgNode object corresponding to the item, it will be displayed on the scene view. (Actually, it is drawn when checked on the item tree view.)

The basic introduction method is the same for other interfaces.

Details on how to introduce RenderableItem and LocatableItem will be explained separately.

.. _plugin-dev-item-type-registration:
 
Registering Item Classes
------------------------

Newly defined item classes need to be registered with the Choreonoid framework.
Only after doing this can they be used in Choreonoid's GUI.

Registration is done using the `ItemManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemManager.html>`_, which performs various management related to item types.
The ItemManager instance used for registration can be obtained with the following function defined in the `ExtensionManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ExtensionManager.html>`_, which is the parent class of the `Plugin class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Plugin.html>`_:

* **ItemManager& itemManager()**

You can call this function from the plugin's initialize function to get the ItemManager instance (reference).
Once you have the instance, you can register item classes with the following member function of ItemManager: ::

 template <class ItemType, class SuperItemType = Item>
 ItemManager& registerClass(const std::string& className);
 
This is a template function where you specify the item class to register as the template argument. And you specify the class name as the function argument. By doing this, various initializations are performed to use the target item class in Choreonoid.

For example, to register the FooItem class: ::

 itemManager().registerClass<FooItem>("FooItem");

If the item class to register inherits from another item class, specify that type as the second template argument of registerClass. For example, if a BarItem class is defined as: ::

 class BarItem : public FooItem
 {
     ...
 };

Register it as: ::

 itemManager().registerClass<BarItem, FooItem>("BarItem");

In this case, FooItem must also be registered beforehand. Note that if FooItem is an abstract class, use the following function for registration: ::

 template <class ItemType, class SuperItemType = Item>
 ItemManager& registerAbstractClass();

In this case, you don't need to specify the item class name.

Abstract class item types cannot be directly created and used, but they can be used when specifying target item types in internal processing. Since some functions may not work if the item class is not registered, register abstract classes in advance with this function.

.. _plugin-dev-singleton-item-registration:

Registering Singleton Items
---------------------------

If an item type allows only one instance, register it as a "singleton item".
Use the following ItemManager function for this: ::

 template <class ItemType, class SuperItemType = Item>
 ItemManager& registerClass(const std::string& className, ItemType* singletonInstance);

In this case, create an instance of the target item type in advance at registration time and specify it as the second argument of this function.
For singleton items, users can only create one instance, and the instance specified here will always be used.

.. _plugin-dev-item-creation-panel-registration:

Registering Creation Panels
---------------------------

Creation panels allow items to be created from the main menu "File" - "New".
Set this with the following ItemManager function: ::

 template <class ItemType>
 ItemManager& addCreationPanel(ItemCreationPanelBase* panel = nullptr);

Using this function, for example: ::

 itemManager().addCreationPanel<FooItem>();

This allows FooItem to be newly created. In this case, a panel for creating Foo items will be displayed via "File" - "New" - "Foo". This panel has default functionality and only allows setting the item name. The panel used is specified by the panel argument, but if this is set to the default value nullptr, the default panel is displayed.

By specifying a custom panel object for the panel argument, you can realize panels with setting items other than the name.
This method is explained in :doc:`item-creation-io-customization` - :ref:`plugin-dev-item-creation-panel-implementation`.