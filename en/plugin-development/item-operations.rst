==========================
Project Item Operations
==========================

.. contents:: Table of Contents
   :local:

Overview
--------

One of the main components of the Choreonoid framework is the "project item". If you have experience using Choreonoid as a user, you should have a general understanding of what these are, but :doc:`../basics/item` also provides a comprehensive explanation for reference.

When trying to implement meaningful functionality in Choreonoid, you will often be working with project items. In plugin development as well, mastering the use of project items is important for achieving your target functionality.

Therefore, this section introduces the "Item class", which is the Choreonoid SDK class corresponding to project items, and explains an overview of how to manipulate project items through this class from plugins.

Note that we will use the abbreviated term "item" instead of "project item" in the following sections.

Item Class
----------

The Choreonoid SDK defines the Item class as the base class for each item. This class defines and implements various information and processing that are fundamental for referencing and manipulating items.

This class is defined in the Base module of the Choreonoid SDK. For details on the class definition, see the API reference for the `Item class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Item.html>`_. In the Choreonoid source repository, it corresponds to the source files `Item.h <https://choreonoid.org/en/documents/reference/latest/Item_8h_source.html>`_ and Item.cpp under src/Base.

The main features implemented in this class are shown below:

* Setting, getting, and change notification for item names
* Setting, getting, and change notification for parent-child/sibling relationships
* Item searching
* Setting, getting, and change notification for item selection/check states

It also provides APIs for uniformly handling the following features implemented in individual item types:

* Item file saving and loading
* Item project saving and loading
* Item property display and editing

In this section, we will introduce operations related to "item names", "parent-child/sibling relationships", "searching", and "selection/check states" from among these.
This should give you an overview of how to manipulate items from programs.
Other topics will be explained later in this development guide.

Item-Derived Classes
--------------------

The Item class is the base class for items, but it does not itself have specific data or functionality to fulfill application purposes. Specific data and functionality are defined and implemented in individual item types that inherit from this Item class.

.. highlight:: text

Various item types are defined in Choreonoid itself as such individual item types. To give you an idea of this, some existing item types are shown below: ::

 + Item
   + WorldItem: Virtual world
   + BodyItem: Physical/rendering models of robots and environments
   + ControllerItem: Base for robot control items
     + SimpleControllerItem: Simple controller implementation
   + SimulatorItem: Base for simulator items
     + AISTSimulatorItem: AIST simulator implementation
     + KinematicSimulatorItem: Kinematic simulator implementation
   + SceneItem: 3D rendering model
   + ScriptItem: Base for script items
     + PythonScriptItem: Implements Python script functionality

Here the inheritance relationships are shown in tree format. As you can see, some inherit directly from the Item class, while other item types have two levels of inheritance. In two-level cases, the first level often defines common APIs for specific functionality, and the second level implements specific implementations based on those APIs. Of course, there is no limit to the depth of inheritance, and examples with three or more levels exist.

In real feature implementation, utilizing the functionality of these individual items is also essential. You can also define your own item types and use them. With this in mind, there are three main levels of item manipulation and utilization:

1. Perform basic item operations using Item class functionality
2. Perform operations related to specific data and processing using individual item type functionality
3. Define and implement your own item types to enable custom data and processing

The content introduced in this section mainly corresponds to level 1. The sample in the next section also uses Body items, which are individual item types, as an example of level 2. Level 3 will be explained in future sections.

.. note:: The explanations and samples from this section onward use Body items that correspond to robot and environment models. We also use related Body models and other objects. We assume that readers of this guide have experience running robot simulations using Choreonoid, in which case you should generally understand Body items and Body models. If not, please refer to :doc:`../handling-models/index` as needed. Your understanding will also deepen as you progress through this guide.

Item Parent-Child/Sibling Relationships
---------------------------------------

In Choreonoid, you generally work by combining multiple items. These multiple items are organized into a tree structure, which is called the :ref:`basics_item_tree`.

Having a tree structure means that parent-child and sibling relationships are built between items. This information is held by the Item class, and you can set and reference relationships using the Item class API.

.. highlight:: cpp

For example, if you have two item instances called itemA and itemB: ::

 itemA->addChildItem(itemB);

This sets itemB as a child item of itemA. This relationship is represented as follows:

.. code-block:: text

 + itemA
   + itemB

.. note:: All items are :doc:`Referenced type <referenced>` objects, dynamically created on heap memory and held by smart pointers ref_ptr. Therefore, coding related to items takes the form of pointers as shown above. Note that items are basically held by ref_ptr references from parent items, and since ref_ptr can be mutually converted with raw pointers, there is no particular problem using raw pointers for temporary references to items.

In this case, the following conditions hold: ::

 itemA->childItem() == itemB
 itemB->parentItem() == itemA

This way you can get the child item or parent item of each item.

At this point, itemA has no parent and itemB has no children, so: ::

 itemA->parentItem() == nullptr
 itemB->childItem() == nullptr

Next, suppose there is also an instance called itemC, which we also add as a child item of itemA: ::

 itemA->addChildItem(itemC);

Then the item tree becomes:

.. code-block:: text

 + itemA
   + itemB
   + itemC

Here itemA has two child items, and a sibling relationship is created between the two child items. Then the following conditions hold: ::

 itemB->nextItem() == itemC
 itemC->prevItem() == itemB

You can see sibling relationships by referencing nextItem and prevItem. Also in this case: ::

 itemB->prevItem() == nullptr
 itemC->nextItem() == nullptr

From the parent item itemA, you can also get the following information: ::

 itemA->lastChildItem() == itemC
 itemA->numChildren() == 2

You can also specify the position to add a child item. In that case, use the insertChild function. For example: ::

 itemA->insertChild(itemC, itemD);

Then the item tree becomes:

.. code-block:: text

 + itemA
   + itemB
   + itemD
   + itemC

Thus itemD was added at the position before itemC. The insertChild function has arguments like: ::

 Item::insertChild(item to be the insertion position, item to insert)

.. note:: If following the same naming as addChildItem, it should be insertChildItem, but here we use a function called insertChild. Actually, there is also a function called insertChildItem, but the argument order is reversed, which is also opposite to the order seen in standard libraries. Since it's common to take the insertion position as the first argument, insertChild was defined as a corrected version, and insertChildItem is now deprecated.

It's common to want to apply certain processing to all child items of an item. You can code this using the childItem and nextItem functions as follows: ::

 for(auto child = itemA->childItem(); child; child = child->nextItem()){
     doSomething(child);
 }

To break a parent-child relationship, use the removeFromParentItem function on the child item. For example: ::

 itemB->removeFromParentItem();

After execution, the item tree becomes:

.. code-block:: text

 + itemA
   + itemD
   + itemC

In this case, sibling relationships involving itemB are also broken. Also, for the parent item itemA: ::

 itemA->clearChildren();

This breaks all child items of itemA.

Root Item
---------

There is always exactly one "root item" in Choreonoid. Items become visible and operable in the GUI by having a connection to the root item. Conversely, items without a connection to the root item are basically not subject to manipulation.

The root item is a singleton instance of the dedicated item type `RootItem class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1RootItem.html>`_. You can get it like: ::

 #include<cnoid/RootItem>

 ...

 auto rootItem = RootItem::instance();

The item tree shown as an example above cannot be handled in the GUI unless it is added to the root item. For example, for the previous tree:

.. code-block:: text

 + itemA
   + itemB
   + itemC

You can connect it to the root item by: ::

 RootItem::instance()->addChildItem(itemA);

In this case the tree becomes: ::

 + RootItem::instance()
   + itemA
     + itemB
     + itemC

And the tree from itemA onward is displayed in the :ref:`basics_mainwindow_itemtreeview`. Conversely, items already loaded in the GUI are connected to the root item in this way.

In this state, the member function isConnectedToRoot() returns true for all items included in itemA's subtree. When not connected to the root item, this returns false.

Whether an item is connected to the root item is an important element in Choreonoid, so please keep this in mind when programming.

.. _plugin-dev-item-basic-attributes:

Basic Item Attributes
---------------------

The Item class holds information about basic item attributes. The attribute items are defined in the Item class enumeration type Attribute as follows:

.. list-table::
 :widths: 40,60
 :header-rows: 1

 * - Attribute
   - Content
 * - SubItem
   - Item that is part of a composite item
 * - Attached
   - Prohibits detachment from parent item
 * - Temporal
   - Temporarily created item
 * - LoadOnly
   - Item that can only be loaded

These attributes can be set and referenced using the following member functions of the Item class:

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - Function
   - Processing
 * - void setAttribute(Attribute attribute)
   - Set the specified attribute
 * - void unsetAttribute(Attribute attribute)
   - Clear the specified attribute
 * - bool hasAttribute(Attribute attribute)
   - Check if the specified attribute is set

The SubItem attribute indicates whether an item is a component of a :ref:`basics_composite_item`. Items with this attribute are called "sub-items". Sub-items cannot change parent-child/sibling relationships, save, load, or delete by themselves. They are always processed as integrated with the main item of the composite item.

This SubItem attribute can also be determined with the dedicated member function isSubItem. Also, when adding an item to a parent item, if you use the addSubItem function instead of the addChildItem function, the item is added as a sub-item.

The Attached attribute is for prohibiting detachment of an item from its parent item. This is similar to the SubItem attribute, but even when set, the item's data and processing remain independent from the parent item, allowing independent saving and loading, for example. Only the operation of detaching from the parent in the GUI is prohibited. This is set when you want to always use an item together with its parent item, even though they are not originally integrated like composite items.

The Temporal attribute indicates that an item was created temporarily. When this attribute is assigned, the item is treated as if it doesn't exist when saving the entire project. That is, it is not saved to the project file, so this item is not restored when the saved project is reloaded, nor is it saved to a file.

This is applied, for example, to items that store simulation result log data. Log data is used to replay simulation results on the spot, but does not necessarily need to be saved as part of the project. You can get the same log by re-running the simulation under the same conditions. Also, log data often becomes huge in size, so trying to save it would make operation cumbersome. For this reason, log data is treated as temporary data with the Temporal attribute.

The LoadOnly attribute is set when an item only supports loading from files and cannot be saved. This attribute should be considered when implementing items and doesn't need special consideration in item operations.

.. _plugin-dev-item-operations-item-list:

ItemList Class
--------------

The `ItemList <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemList.html>`_ template class is defined as a container for storing multiple Items. This is a type of array that stores item pointers (specifically smart pointers), but it becomes an array that selectively stores only items of the type given as the template argument.

This is used to extract only items of a specified type from functions that return multiple items.

First, include the header for this class: ::

 #include <cnoid/ItemList>

Here, let's assume there is a hypothetical function: ::

 ItemList<Item> getItemList();

The return value of this function is an ItemList for Item type. That is, this list can store all item types. By the way, since the template argument of ItemList defaults to the Item class, a list for the Item class like this example can be written as ItemList<>. We'll use that notation below.

Here, if you write: ::

 ItemList<> items = getItemList();

items will contain the same results as the item set returned by getItemList. However, if you write: ::

 ItemList<BodyItem> bodyItems = getItemlist();

Only items that match the BodyItem type among those returned by getItemList will be stored in bodyItems.

By using ItemList with a specific type specified in this way, you can selectively get only items that match that type. Between ItemLists, even if they target different types, you can use copy constructors and assignment operators mutually. In that case, what is actually copied is only the item types targeted by the newly created ItemList or the assignment destination ItemList.

The Choreonoid SDK has many functions that return ItemList, which can be conveniently used in actual usage. Specific examples are introduced below.

.. note:: Elements of ItemList are ref_ptr that hold items of the specified type. Therefore, items are guaranteed to remain alive while contained in the ItemList.

.. _plugin-dev-item-detection:

Item Search
-----------

You can get items on the item tree that match given conditions.

A commonly used example is extracting items that match a certain item type from the tree. For example: ::

 ItemList<BodyItem> bodyItems = item->descendantItems<BodyItem>();

This extracts BodyItem type items from the item tree starting from item and returns them as an ItemList. descendantItems is a function that gets elements of the subtree starting from the target item. By specifying a particular item type as its template argument, only items of that type are extracted. This example can be used when you want to perform certain operations on all BodyItems in the subtree. If you make the root item the starting point, all BodyItems existing in the project are extracted.

You can also search for items by name. For the item that is the starting point of the search: ::

 Item* robotItem = item->findChildItem("Robot");

This searches for the first item with the name "Robot" among item's child items and returns it if it exists. If the item doesn't exist, nullptr is returned.

This function can also specify hierarchical names. For example: ::

 Item* controllerItem = item->findChildItem("World/Robot/Controller");

Starting from item, if it can reach Controller with the following structure, it returns it. The hierarchical name specified here can be rephrased as a "search path".

.. code-block:: text

 + item
   + "World"
     + "Robot"
       + "Controller"

This function also has a template version. For example: ::

 BodyItem* robotItem = item->findChildItem<BodyItem>("Robot");

This returns a BodyItem type child item with the name "Robot" if it exists. In this case, even if an item with the same name exists, nullptr is returned if the type is different.

There is also a findItem function that is similar to findChildItem but behaves slightly differently. It matches even if the search path doesn't necessarily start from the target item. For example, with the previous search path: ::

 Item* controllerItem = item->findChildItem("Robot/Controller");

This returns nullptr because there is no Robot item directly under item's child items, but if you use: ::

 Item* controllerItem = item->findItem("Robot/Controller");

It returns Controller.

This findItem function also has a template version like findChildItem. The usage is the same.

The above functions can also specify arbitrary search conditions based on their respective basic search methods. In that case, search conditions are given as function objects that take an item as an argument and return a bool value.

For example: ::

 ItemList<> = item->descendantItems([](Item* item){ return item->isSubItem(); });

This returns items with the SubItem attribute under item. In findChildItem and findItem, you can replace the search path with arbitrary search conditions or specify search conditions as additional arguments. For details, see the `Item class reference <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Item.html>`_.

We've introduced functions that search within a subtree starting from a certain item, but there is also a function called findOwnerItem that searches by traversing parent items from a certain item.

For example, if you want to know which WorldItem a certain BodyItem belongs to: ::

 WorldItem* worldItem = bodyItem->findOwnerItem<WorldItem>();

This searches for parent items from bodyItem and returns a WorldItem type item when found. If not found, it returns nullptr.

This feature is actually frequently used in implementing various Choreonoid features.
Since Choreonoid is basically designed to judge relationships between items by their parent-child relationships, processing of an item is often done in cooperation with its parent (or ancestor) items, in which case you can search for cooperating items using the above method.

Item Selection/Check States
---------------------------

Items have "selection state" and "check state". If you have experience using Choreonoid, you should already understand what these states are. If not, please refer to :ref:`basics_itemtree_management` - :ref:`basics_selection_and_check`.

These states can of course be set and retrieved from programs. First, the following are available as member functions of the Item class:

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - Function
   - Processing
 * - bool isSelected() const
   - Returns the current selection state
 * - void setSelected(bool on, bool isCurrent = false)
   - Switches to the selection state specified by on
 * - void setSubTreeItemsSelected(bool on)
   - Batch switches the selection state of items in the subtree including itself
 * - bool isChecked(int checkId = PrimaryCheck) const
   - Returns the current check state
 * - void setChecked(bool on)
   - Switches to the check state specified by on

These functions allow you to set or get the selection/check state of items. When you call the above setting functions from a program, the GUI state also switches immediately.

.. note:: The arguments isCurrent and checkId in the above functions are used for somewhat advanced usage methods. If there's no particular reason, it's fine to use them with default arguments.

To get the selection state within the entire item tree, use RootItem functions.

First, to get all selected items on the item tree: ::

 ItemList<> selectedItems = RootItem::instance()->selectedItems();

This function also has a template version that specifies the item type. Using it, for example: ::

 ItemList<BodyItem> selectedBodyItems = RootItem::instance()->selectedItems<BodyItem>();

You can get the currently selected Body items.

The same applies to check states: ::

 ItemList<> checkedItems = RootItem::instance()->checkedItems();

Or specifying the item type: ::

 ItemList<BodyItem> checkedBodyItems = RootItem::instance()->checkedItems<BodyItem>();

Referencing item selection and check states is often used to determine which items are targets for various functions. Please make appropriate use of these states in developing your own plugins.

.. _plugin-dev-item-operations-signals:

Item-Related Signals
--------------------

Signals are defined for changes in the item tree and changes in item states. Using these signals, you can perform appropriate processing on items at appropriate times and appropriately switch target items for various functions.

First, signals that notify changes in individual items are defined as members of the Item class. The available ones are listed below. All are defined as member functions that return SignalProxy.

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - Signal
   - Timing of emission
 * - SignalProxy<void(const std::string& oldName)> sigNameChanged()
   - When its own name is changed
 * - SignalProxy<void()> sigTreePathChanged()
   - When the path from root to itself in the item tree changes (due to addition, movement, deletion, etc.)
 * - SignalProxy<void()> sigTreePositionChanged()
   - When its own position in the item tree changes (due to addition, movement, deletion, etc.). Position changes include path changes plus changes in order among sibling items.
 * - SignalProxy<void()> sigSubTreeChanged()
   - When the composition of its own subtree changes (due to item addition, movement, deletion, etc.)
 * - SignalProxy<void()> sigDisconnectedFromRoot()
   - When the connection between itself and the root item is broken
 * - SignalProxy<void(bool on)> sigSelectionChanged()
   - When its own selection state changes
 * - SignalProxy<void(bool on)> sigCheckToggled(int checkId = PrimaryCheck)
   - When its own check state changes
 * - SignalProxy<void()> sigUpdated()
   - When its own content is updated

The following signals defined in the RootItem class are also available. All notify changes within the item tree starting from the root item, and items not connected to the root item are not subject to these signals.

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - Signal
   - Timing of emission
 * - SignalProxy<void(Item* item)> sigSubTreeAdded()
   - When a subtree starting from item is added
 * - SignalProxy<void(Item* item)> sigItemAdded()
   - When an item is added. At the same timing as sigSubTreeAdded, this signal is emitted for each item in the added subtree.
 * - SignalProxy<void(Item* item)> sigSubTreeMoved()
   - When a subtree starting from item is moved
 * - SignalProxy<void(Item* item)> sigItemMoved()
   - When an item is moved. At the same timing as sigSubTreeMoved, this signal is emitted for each item in the moved subtree.
 * - SignalProxy<void(Item* item, bool isMoving)> sigSubTreeRemoving()
   - Just before a subtree starting from item is deleted. This signal is also emitted when the subtree is moved. In that case, it is emitted before sigSubTreeMoved, and isMoving becomes true.
 * - SignalProxy<void(Item* item, bool isMoving)> sigSubTreeRemoved()
   - When a subtree starting from item is deleted. This signal is only emitted when completely deleted, not when moved.
 * - SignalProxy<void(Item* item, const std::string& oldName)> sigItemNameChanged()
   - Similar signal to Item class's sigNameChanged. This one allows connecting slots without limiting target items
 * - SignalProxy<void(const ItemList<>& selectedItems)> sigSelectedItemsChanged()
   - When selected items change
 * - SignalProxy<void(Item* item, bool on)> sigCheckToggled(int checkId = PrimaryCheck)
   - When checked items change

All are signals utilized in the implementation of Choreonoid itself.

Let me introduce usage guidelines for some signals.

First, Item class's sigTreePathChanged is used to perform preparation or cleanup for processing when the relationship with items above (parent side) the target item affects the processing content. This is also emitted at the timing of connecting to the root item, so it's often used to initialize items at that time.

On the other hand, when the relationship with items below (child side) the target item affects the processing content, sigSubTreeChanged is used to detect relationship changes and perform preparation or cleanup for processing.

Item class's sigDisconnectedFromRoot means that the item will no longer be a target for operations in Choreonoid after that, so it's used for item cleanup. For example, when an item is using Choreonoid objects or OS resources, it performs their release processing.

State detection of item selection and check states is also often done through signals. Item class's sigSelectionChanged and sigCheckToggled can be used to detect state changes of specific instances, and if you want to detect changed items within the entire item tree, use RootItem class's sigSelectedItemsChanged and sigCheckToggled.

In RootItem's sigSelectedItemsChanged, a list of selected items is given as an ItemList. By passing this list to an ItemList for a specific item type, you can detect selection state changes for that item type. For example, define a function to process selection state changes as: ::

 void onSelectedBodyItemsChanged(ItemList<BodyItem> selectedBodyItems)
 {
     ...
 }

And then: ::

 RootItem::instance()->sigSelectedItemsChanged().connect(
     [](const ItemList<>& selectedItems){
         onSelectedBodyItemsChanged(selectedItems);
     });

Then onSelectedBodyItemsChanged receives an ItemList containing only the selected BodyItems.

.. Should TargetItemPicker be explained here too?