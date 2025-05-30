==============================
Project Item State Saving
==============================

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

Overview
--------

As explained in :doc:`../basics/item`, Choreonoid can save project item configurations, states, various data, and the states of various interfaces such as related toolbars and views all together in a project file. By loading the saved project file, you can restore the entire project state and continue your work.

This section explains how to enable saving project item states to project files.

.. _plugin-dev-state-store-restore-functions:

State Store/Restore Functions
-----------------------------

The `Item class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Item.html>`_ defines the following virtual functions for saving and restoring item states to/from project files:

* **virtual bool store(Archive& archive)**

  * Saves the item state.

* **virtual bool restore(const Archive& archive)**

  * Restores the item state.

By overriding and implementing these functions in each item class, you can support state saving and restoration for items.
Each should return true on successful processing and false on failure.
When false is returned, saving to or restoration from the project file is skipped.

These functions exchange information about item states through the archive argument of `Archive type <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Archive.html>`_. Details about this will be described later.

Similar to :ref:`plugin-dev-put-property-function`, when an item type implementing these functions inherits from another item type (not the Item class), you usually need to call the same functions of the parent class. For example, for BarItem inheriting from FooItem, the store and restore functions should be implemented in the following form: ::

 bool BarItem::store(Archive& archive)
 {
     if(FooItem::store(archive)){
         // BarItem's store processing
         ...
         return true;
     }
     return false;
 }

 bool BarItem::restore(const Archive& archive)
 {
     if(FooItem::restore(archive)){
         // BarItem's restore processing
         ...
         return true;
     }
     return false;
 }

.. _plugin-dev-yaml-structured-data-classes:

YAML-Type Structured Data Classes
---------------------------------

The `Archive class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Archive.html>`_, which is the argument for state store/restore functions, inherits from the `Mapping class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Mapping.html>`_ and adds functions related to project saving/restoration. The Mapping class is part of a group of classes called "structured data classes," through which various data can be stored in a structured manner.

`YAML <http://yaml.org/>`_, which prefixes the "YAML-type structured data classes," is a general-purpose data description language for describing structured data and objects in text. It is primarily defined for serializing data and objects, and is simple, highly readable, supported by various programming languages, and widely used today. Text written in YAML is often saved to files for use. These files are called YAML files.

When using the "YAML-type structured data classes" including Archive, it's desirable to first have basic knowledge about YAML. If you don't have this knowledge, please refer to resources like the following to understand the basic specifications and usage:

* `Wikipedia YAML page <https://en.wikipedia.org/wiki/YAML>`_

* `Learn X in Y minutes - YAML <https://learnxinyminutes.com/yaml/>`_

The "YAML-type structured data classes" express YAML structures in C++ classes and are defined and implemented in the Util library of the Choreonoid SDK. Parsers that read YAML text to generate objects of these classes and writers that output objects of these classes as YAML text are also available, making it easy to read and write YAML files.

This class group consists of the following classes:

* `Mapping <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Mapping.html>`_

  * Corresponds to YAML Mapping (associative array, hash)
  * Can store multiple key-value pairs
  * Each value is an object of either Mapping, Listing, or ScalarNode
  * MappingPtr is defined as the corresponding smart pointer type

* `Listing <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Listing.html>`_

  * Corresponds to YAML Sequence (array)
  * Can store multiple values as array elements
  * Each value is an object of either Mapping, Listing, or ScalarNode
  * ListingPtr is defined as the corresponding smart pointer type

* `ScalarNode <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ScalarNode.html>`_

  * Corresponds to YAML scalar values
  * Stores one of bool, int, double, or string as a scalar value
  * ScalarNodePtr is defined as the corresponding smart pointer type

All inherit from the `ValueNode class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ValueNode.html>`_ as the base class. Since ValueNode inherits from Referenced, all the above classes are :doc:`referenced`. This class hierarchy is illustrated as follows:

.. code-block:: text

 + Referenced
   + ValueNode
     + Mapping
     + Listing
     + ScalarNode

These classes are defined and implemented in ValueTree.h and ValueTree.cpp in the Util library, and can be used in the Choreonoid SDK by including the ValueTree header.
    
Let's introduce an example of data construction using these classes. For example, suppose there is data described in YAML as follows:

.. code-block:: yaml

 color: red
 height: 1.8
 translation: [ 0.0, 1.0, 2.0 ]

The corresponding data can be constructed using structured data class objects as follows:

* **Mapping**

  * Key: color

    * Value: **ScalarNode("red")**

  * Key: height

    * Value: **ScalarNode(1.8)**

  * Key: translation

    * Value: **Listing**

      * Value: **ScalarNode(0.0)**

      * Value: **ScalarNode(1.0)**

      * Value: **ScalarNode(2.0)**

The parts in bold are objects of the above classes.
(The key parts are not objects by themselves but are part of the Mapping object.)
These objects are called "nodes" in the data structure.
Multiple nodes form a tree structure with hierarchical parent-child relationships.
Strictly speaking, it's a graph structure since a node can be shared by multiple parent nodes.

In this example, the top-level Mapping node corresponds to the entire data.
From there, each value is held hierarchically as nodes.

The C++ code to generate this data can be written as follows: ::

 #include <cnoid/ValueTree>
 ...
 
 // Generate the top-level Mapping object
 MappingPtr data = new Mapping;
 // Add key-value pairs (ScalarNode) to the node
 data->write("color", "red");
 data->write("height", 1.8);
 // Add a Listing node as a value
 auto translation = data->createListing("translation");
 // Add elements (ScalarNode) to the Listing node
 translation->append(0.0);
 translation->append(1.0);
 translation->append(2.0);

For constructing the translation node, if the value is stored in a three-dimensional vector type Vector3, you can write as follows using functions from the EigenArchive header: ::

 #include <cnoid/EigenArchive>
 ...

 Vector3 translation;
 ...

 write(data, translation);

This data can be output as a YAML file using the `YAMLWriter class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1YAMLWriter.html>`_. This is done as follows: ::

  #include <cnoid/YAMLWriter>
  ...

  YAMLWriter writer("data.yaml")
  writer.putNode(data);

Conversely, you can also read a YAML file to construct structured data. This is done using the `YAMLReader class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1YAMLReader.html>`_ as follows: ::

  #include <cnoid/YAMLReader>
  ...

  YAMLReader reader;
  MappingPtr data;
  try {
      data = reader.loadDocument("data.yaml")->toMapping()
  }
  catch(const ValueNode::Exception& ex){
      ...
  }

In this case, if reading succeeds, a Mapping type object is assigned to the variable node.
If there's a problem with the YAML file, a ValueNode::Exception type exception is thrown.

Code to read data expecting the above structure to be stored in a Mapping object can be written as follows: ::

  std::string color;
  double height;
  Vector3 translation;

  data->read("color", color);
  data->read("height", height);

  // Reading the 3 elements of translation
  auto translationNode = data->findListing("translation");
  if(translationNode->isValid()){
      if(translationNode->size() == 3){
          for(int i=0; i < 3; ++i){
              translation[i] = translationNode->at(i)->toDouble();
          }
      }
  }

In this example, if the data has the expected structure, the read values are assigned to the variables color, height, and translation.

By using the get function instead of the read function, you can read with default values specified. For example: ::

  std::string color = data->get("color", "red");
  double height = data->get("height", 1.8);

If the top-level node doesn't contain the keys color or height, "red" and 1.8 are returned as default values, respectively.
  
Reading translation can also be written in one line using functions from the EigenArchive header: ::

 #include <cnoid/EigenArchive>
 ...
 
 read(data, "translation", translation);

Thus, by using the YAML-type structured data classes and related classes, you can read and write structured data with the same structure as YAML. Each class in the structured data classes has various member functions for reading and writing, allowing flexible coding for reading and writing. Functions like those in the EigenArchive header are also provided to concisely write reading and writing for specific types. For details about these, please refer to the API reference manual. Also, parts of Choreonoid's source code that implement store and restore functions can serve as references for usage.

.. _plugin-dev-archive-class:

Archive Class
-------------

As seen in previous examples, YAML-type structured data often uses Mapping as the top-level node, and Mapping plays a central role in reading and writing data. Therefore, it's conceivable to read and write states through Mapping class objects in item state store/restore functions.

However, since the Mapping class is a general-purpose class for storing structured data, it may not have all the functions necessary for project saving and restoration. To consolidate these into a single argument for a simpler API, the `Archive class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Archive.html>`_ is defined. This adds functions related to project saving and restoration to the Mapping class.

Please refer to the `Archive class reference <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Archive.html>`_ for the added functions. Below, we introduce the main functions that can be used in implementing project item state saving/restoration by category.

.. _plugin-dev-archive-post-processing:

Functions Related to Post-Processing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As described in :ref:`plugin-dev-project-file-structure`, project files record the states of various objects related to the project, including items, which are read sequentially. Among these, some object states may depend on other objects. However, when reading such objects, the dependent objects may not have been read yet. In that case, you need to wait for the dependent objects to be read before reading the related states.

Such processing is called "post-processing" of state restoration. The following Archive class functions enable this post-processing:

* **void addProcessOnSubTreeRestored(const std::function<void()>& func) const**

  * Executes the specified function when all items in the subtree of the currently loading item have been loaded.

  * Used when an item's state depends on other items within its own subtree.

* **void addPostProcess(const std::function<void()>& func, int priority = 0) const**

  * The specified function is executed after all objects in the project have been loaded.

  * When there are multiple post-processing functions, the execution order is determined by the priority argument. The default value when omitted is 0, and smaller priority values are executed first.

* **void addFinalProcess(const std::function< void()>& func) const**

  * The specified function is called when all post-processing executed by addPostProcess is completed.

These are basically used within an item's restore function.
For addPostProcess and addFinalProcess, they can be used recursively within post-processing functions to perform further post-processing.

.. _plugin-dev-archive-item-reference:

Functions Related to Item References
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following functions related to referencing other items included in the project are available:

* **Item* currentParentItem() const**

  * Returns the parent item of the currently loading item during project loading. Used when parent information is needed for state restoration.

  * Since items are added to parent items after loading is completed (successful), you cannot reference your own parent during loading. This function allows you to reference the item that will become the parent.

* **ValueNodePtr getItemId(Item *item) const**

  * Gets the ID of an item included in the same project. Used to record references to other items during project saving.

  * The ID is usually a scalar node (ScalarNode) containing an integer value, but for sub-items that constitute :ref:`basics_composite_item`, it becomes a Listing containing "the integer ID value of the main item" + "item name(s) leading to the sub-item (multiple possible)".

* **Item* findItem(ValueNodePtr id)const**

  * Gets an item included in the same project by specifying its ID. Used to resolve references to items during project loading. Returns nullptr if no item corresponding to the ID is found.

  * The ID has the same format as the return value of getItemId.

  * A template version that can specify the type of item to retrieve is also available.

Here's an example of using getItemId and findItem. Suppose FooItem holds a pointer to BarItem and we want to restore this when loading the project: ::

 class FooItem : public Item
 {
     BarItem* barItem;

 public:
     ...

     virtual bool store(Archive& archive) override;
     virtual bool restore(const Archive& archive) override;
 };

In this case, the store function would be as follows: ::

 bool FooItem::store(Archive& archive)
 {
     auto id = archive.getItemId(barItem);
     archive.write("bar_item_id", id);
     ...
     return true;
 }

This writes the BarItem's ID with the key "bar_item_id" as FooItem's state data.

The restore function would be implemented as follows: ::

 bool FooItem::restore(const Archive& archive)
 {
     archive.addPostProcess(
         [this, &archive](){ barItem = archive.findItem<BarItem>(archive.find("bar_item_id"); });
     ...
     return true;
 }

Here we use addPostProcess to resolve the reference in post-processing. This is necessary when you don't know where BarItem exists or when it's loaded after FooItem. This is because BarItem doesn't necessarily exist when FooItem is loaded.

Alternatively, if you know that BarItem exists within FooItem's subtree, you can use addProcessOnSubTreeRestored instead of addPostProcess. If BarItem definitely exists above FooItem, you can resolve the reference directly within the restore function.

We use the template version of the findItem function to get the item corresponding to the ID.
This directly obtains a BarItem type object.

.. note:: It's usually better not to directly resolve references to other items using this method. Instead, it's more desirable to resolve references based on parent-child relationships between items, which is the standard method in Choreonoid. Reference resolution by item ID is rather used by other types of objects like views to resolve references to related items.

.. _plugin-dev-relocatable-filepath-functions:

Functions Related to Relocatable File Paths
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Some project items load data from files.
In that case, items need to hold file-related information such as file paths and file formats, and this information is also recorded during project saving.
A point to note is that files may be placed in various directories in the file system, and these may be in different locations when the environment or user changes.
To be able to restore the original project even in such cases, it's desirable to record each file's path in a portable format. We call file paths in such a format "relocatable file paths."
The Archive class also provides functions to use this format.

First, it provides the following functions to get relocatable file paths. These are mainly used from the store function:

* **std::string getRelocatablePath(const std::string& path) const**

  * Converts any file path string to a relocatable file path.
 
* **bool writeRelocatablePath(const std::string& key, const std::string& path)**

  * Converts any file path to a relocatable file path and writes it with the specified key.

Relocatable file paths actually consist of the following elements:

1. Relative path from the project directory
2. Path variables
3. User variables

For 1, if a file is in the project file's save destination directory (called the project directory) or its subdirectories, it's described as a relative path from the project directory. For example, suppose the project file "simulation.cnoid" and other files used in the project are arranged in the following directory structure:

.. code-block:: text

 + home
   + choreonoid
     + project
       - simulation.cnoid
       - robot.body
       + data
         - command.dat

Here:

* Project directory

  * **/home/choreonoid/project**

* Relative paths from the project file for each file

  * **robot.body**

  * **data/command.dat**

In this case, even if you move the project directory to another location, if its contents haven't changed, when loading the project "simulation.cnoid", you can determine the locations of "robot.body" and "command.dat".

As a slightly different situation, suppose you're using model files that come with Choreonoid itself. These are under Choreonoid's installation share directory, so for example, the structure would be:

.. code-block:: text

 + home
   + choreonoid
     + project
       - simulation.cnoid
       - robot.body
       + data
         - command.dat
 + usr
   + local
     + share
       + choreonoid-1.8
         + model
           + misc
             - floor.body

Here we assume Choreonoid is installed in "/usr/local" and we're using a model called "floor.body" contained within it.

In this situation, you can use "path variables" mentioned in 2 above. Using these, the path to floor.body

* **/usr/local/share/choreonoid-1.8/model/misc/floor.body**

can be written as:

* **${SHARE}/model/misc/floor.body**

The ${SHARE} part is a "path variable" corresponding to the share directory. Even if the environment changes and Choreonoid's actual installation location or version changes, this always points to the share directory in that environment. Therefore, if the file path is recorded in this format, the project can be loaded in any environment.

The following path variables are available:

* **PROGRAM_TOP**

  * Top directory of Choreonoid installation

* **SHARE**

  * Share directory of Choreonoid installation

* **HOME**

  * Home directory of the current user

These path variables are applied when converting to relocatable file paths if possible. In that case, the path variable that makes the path shortest (closest to the target file) is automatically assigned. However, when the project directory is closest, "relative path from project directory" takes priority.

Furthermore, as "user variables" mentioned in 3 above, users can define their own equivalents to path variables. For example, assume the following file structure:

.. code-block:: text

 + home
   + choreonoid
     + project
       - simulation.cnoid
       + data
         - command.dat
     + model
       + robot
         - robot.body
 + usr
   + local
     + share
       + choreonoid-1.8
         + model
           + misc
             - floor.body

Previously, "robot.body" was stored in the project directory, but if this is a robot model, you may want to use it from other projects. In that case, rather than creating copies in each project directory, it's more efficient to place it in a directory independent of projects and share it among multiple projects. So in the above structure, "robot.body" is stored in an independent directory for model storage.

In this case, you could write it as

* **${HOME}/model/robot/robot.body**

using the path variable HOME, but suppose you want to freely decide the placement of model files for each environment and user. In that case, you can define a user variable like:

* Variable: **MODEL**

* Path: **/home/choreonoid/model**

Then the path can be written as:

* **${MODEL}/robot/robot.body**

And if you set the path variable MODEL for each user, you can load the project even if its location changes.

User variable settings can be configured from Choreonoid's GUI. For this, please refer to :doc:`../basics/config` - :ref:`basics_project_pathset`.

To get the actual file path from a relocatable file path, use the following functions. These are mainly used from the restore function:

* **std::string resolveRelocatablePath(const std::string& relocatable, bool doAbsolutize = true) const**

  * Converts the relocatable file path given as an argument to an actual file path.

  * Returns an empty string if conversion fails.

  * If doAbsolutize is true, ensures it becomes an absolute path (full path). If false, it won't necessarily be an absolute path. (If you give a relative path without path variables, the result will also be a relative path.)

* **bool readRelocatablePath(const std::string& key, std::string& out_value) const**

  * Retrieves the value of the key specified by key as a relocatable file path, converts it to an actual path with resolveRelocatablePath, and sets the result in out_value. The set path will be an absolute path.

.. _plugin-dev-file-io-functions:
    
Functions Related to File Input/Output
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Archive class also provides the following functions to support file reading:

* **bool writeFileInformation(Item* item)**

  * Writes file path and file format information recorded in the item in a prescribed format.

* **bool loadFileTo(Item* item) const**

  * Reads file path and file format information written in the prescribed format and actually loads the corresponding file into the item.

The usage of these functions is explained in the "Project Item File Saving" section.

.. _plugin-dev-project-file-structure:

Project File Structure
----------------------

Data output to archive by the item class's store function is ultimately saved as a YAML project file.
This is usually saved with the extension cnoid.

This project file records the following information:

* Item states

* View states

* Toolbar states

* Other object states

* View layout

* Toolbar layout

※ View and toolbar layouts are only effective when "Layout" is checked in the main menu's "File" - "Project File Options" - "Layout", as introduced in :ref:`basics_layout_save`.

Among the above, "item states" are the target of the item class's store and restore. Other information is saved and restored by similar functions of the corresponding objects. These will be explained separately.

Since YAML used for describing project files is a highly readable format, if you open a project file in a text editor, you can generally understand its contents. Project files are solely generated by Choreonoid's project saving and are not intended to be directly generated or edited, but we hope you'll make flexible use of YAML's advantages as needed.