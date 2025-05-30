======================================
Customizing Item Creation and File I/O
======================================

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

Overview
--------

Interfaces related to project item creation and file input/output can be customized for each item type.
This allows you to add initialization processing and options specific to certain item types, improving usability.
This section explains how to perform such customization.

.. _plugin-dev-item-creation-panel-implementation:

Customizing Creation Dialog with ItemCreationPanel
--------------------------------------------------

In :doc:`new-item-type`, we showed that by performing :ref:`plugin-dev-item-creation-panel-registration`, items can be created from the "File" - "New" menu. However, the method introduced there uses the default creation panel, where the only configurable setting during creation is the item name, and all other elements are created in their default state. However, depending on the item type, you may want to preset items other than the name during creation, or perform some initialization processing. This can be achieved by registering a custom creation panel.

Creation panels are implemented by inheriting from the `ItemCreationPanel class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemCreationPanel.html>`_. This class is defined in the ItemManager header as follows: ::

 class ItemCreationPanel : public QWidget
 {
 public:
     ItemCreationPanel();
     virtual bool initializeCreation(Item* protoItem, Item* parentItem) = 0;
     virtual bool updateItem(Item* protoItem, Item* parentItem) = 0;
 };

As shown in this definition, ItemCreationPanel inherits from Qt's QWidget class, which serves as the base for placing GUI elements in the creation panel.
Virtual functions for processing item creation are defined, and by implementing these functions in derived classes, you can add arbitrary configuration items and initialization processing to the creation panel.

The protoItem defined as the first argument of each virtual function corresponds to the item's "prototype."
This is an instance of the target item type, usually created using the item's default constructor.
The prototype is held for each creation panel, and item creation is realized by duplicating this prototype.
By changing the prototype's settings, they are reflected in the created items.
Even when creation is performed repeatedly, the same prototype instance is retained, and the prototype's last settings are carried over as defaults for the next creation.

With this prototype item as the axis, the creation panel processes in the following flow:

1. Build the panel's GUI in the constructor
2. The initializeCreation function is called when the panel is displayed, so initialize the prototype item and configuration item GUI there
3. The updateItem function is called just before item creation, so update the prototype item's settings by referencing input to the configuration item GUI
4. Create the item by duplicating the prototype item

Implement the processing for steps 1-3 above in a class that inherits from ItemCreationPanel. For functions 2 and 3, return true on success and false on failure.

Note that to make it easier to implement panels for target item types, the following template class `ItemCreationPanelBase <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemCreationPanelBase.html>`_ is defined: ::

 template<class ItemType>
 class ItemCreationPanelBase : public ItemCreationPanel
 {
 protected:
     ItemCreationPanelBase() { }
     virtual bool initializeCreation(ItemType* protoItem, Item* parentItem) = 0;
     virtual bool updateItem(ItemType* protoItem, Item* parentItem) = 0;
 };

Using this template, you can make the prototype item argument of the virtual functions you override the target item type.
For example, when implementing a creation panel for FooItem: ::

 class FootItemCreationPanel : public ItemCreationPanelBase<FooItem>
 {
 public:
     FootItemCreationPanel();
 protected:
     virtual bool initializeCreation(FooItem* protoItem, Item* parentItem) override;
     virtual bool updateItem(FooType* protoItem, Item* parentItem) override;
 };
     
In this case, the type of the protoItem argument of the above virtual functions becomes a pointer to ProtoItem.
     
For registering creation panels, as introduced in :ref:`plugin-dev-item-type-registration`, use the following function of the `ItemManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemCreationPanel.html>`_: ::

 template <class ItemType>
 ItemManager& addCreationPanel(ItemCreationPanel* panel = nullptr);

Specify an instance of the creation panel in the panel argument. With the default value of nullptr, the default creation panel is used, but if you specify a custom creation panel implemented as described above, that will be used instead.

.. _plugin-dev-itemfileio:

Customizing Input/Output with ItemFileIO
----------------------------------------

The file input/output dialogs shown in :doc:`item-file-io` only specify the target file.
However, there are cases where you want to make additional specifications related to other elements during file input/output.
For example, there may be parts of a file format specification that are not strictly defined, and you may want to ask for user instructions on how to handle those parts. To do this, you need to add GUI elements for option settings to the file input/output dialog and reflect operations on them in the file input/output processing.

Such customization of file input/output can be achieved by registering `ItemFileIO <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemFileIO.html>`_ instead of :ref:`plugin-dev-item-io-function-registration`. ItemFileIO is a class for implementing item file input/output, and using it enables more detailed input/output implementation than :ref:`plugin-dev-item-io-function-registration`. In fact, all file input/output is internally processed as ItemFileIO, and file input/output functions are actually converted to ItemFileIO internally during registration.

ItemFileIO is defined in the Base module, and a header with the same name is provided. Input/output implementation is done in classes that inherit from ItemFileIO.
The flow of this implementation is shown below:

1. Define a custom ItemFileIO class that inherits from ItemFileIO for the target item type
2. Set basic attributes of file input/output in the constructor
3. If supporting file input (loading), override and implement functions for loading
4. If supporting file output (saving), override and implement functions for saving
5. If providing input/output options, override and implement related functions
6. Register with ItemManager's registerFileIO function

Below we explain each of the above items.

Defining Custom ItemFileIO Classes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Item 1 above is basically defined by inheriting from ItemFileIO as follows: ::

 class FooItemFileIO : public ItemFileIO
 {
 public:
     FooItemFileIO();
     ...
 };

Here we assume an ItemFileIO targeting FooItem.

Using the `ItemFileIoBase template <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemFileIoBase.html>`_ makes it easier to implement ItemFileIO specialized for a certain item type. It is used as follows: ::

 class FooItemFileIO : public ItemFileIoBase<FooItem>
 {
 public:
     FooItemFileIO();
     ...
 };
  
In this case, for virtual functions that take target items as arguments, the argument type becomes a pointer to the target item type.
Usually, this method is recommended.

As a somewhat special case, it's also possible to create extensions of existing ItemFileIO.
In this case, use the `ItemFileIoExtender template <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemFileIoExtender.html>`_ as follows: ::

Setting Basic Attributes
~~~~~~~~~~~~~~~~~~~~~~~~

As shown in item 2 above, set basic attributes of file input/output in the constructor of your custom ItemFileIO class.

First, the constructor of the base class (ItemFileIO or ItemFileIoBase template) is defined as follows, so specify the file format and supported APIs with these arguments: ::

  ItemFileIO::ItemFileIO(const std::string& format, int api);

  ItemFileIoBase::ItemFileIoBase(const std::string& format, int api)

Specify a string (identifier) representing the file format in format. This is the same as used in :ref:`plugin-dev-item-io-function-registration`. Also, specify the supported APIs in api as a combination of the following symbols defined in ItemFileIO's enumeration type "API":

* **Load**

  * Supports file loading

* **Save**

  * Supports file saving

* **Options**

  * Supports options

* **OptionPanelForLoading**

  * Supports option setting panel in file loading dialog

* **OptionPanelForSaving**

  * Supports option setting panel in file saving dialog

For example, for FooItemFileIO inheriting from the ItemFileIoBase template, specify as follows: ::

 FooItemFileIO::FooItemFileIO()
     : ItemFileIoBase<FooItem>("FOO-DATA-FILE", Load | Options | OptionPanelForLoading)
 {
     ...
 }

In this case, the file format is "FOO-DATA-FILE", and loading, options, and the option setting panel in the loading dialog are supported.
To support saving, similarly specify Save or OptionPanelForSaving.
At minimum, you need to support either the Load or Save API, but support for the remaining APIs is optional. It's possible to support all APIs.
 
In the constructor implementation, you can set various attributes using the following functions of the ItemFileIO class:

* **void setCaption(const std::string& caption)**

  * Sets the input/output caption. Used in titles of input/output dialogs, etc.

  * Basically represents "what content is being input/output" and doesn't necessarily depend on the file format. For example, there are multiple file formats that can be loaded as body models, but the caption set with this function is "Body" for all formats.

* **void setFileTypeCaption(const std::string& caption)**

  * Sets the file type caption. Displayed as a file type choice in input/output dialogs.

  * If not set with this function, the content set with setCaption is also used as the file type caption.

* **void setExtension(const std::string& extension)**

  * Sets the file extension for the target file format.

* **void setExtensions(const std::vector<std::string>& extensions)**

  * Sets multiple file extensions.

* **void setInterfaceLevel(InterfaceLevel level)**

  * Sets the interface level when used.

  * The value is one of the following defined in ItemFileIO's enumeration type "InterfaceLevel":

  * **Standard**
 
    * Standard level. Added to file load/save items. This is the default setting.
       
  * **Conversion**

    * Conversion level. Added to file import/export items.
	 
  * **Internal**

    * Internal use level. Cannot be directly used by users from menus, etc., and is limited to use from program code.

* **void addFormatAlias(const std::string& format)**

  * Adds a file format alias.

  * Even if you change the file format identifier, by registering the previous identifier as an alias, you can load project files saved with the previous identifier. Mainly used to ensure backward compatibility.


Implementing File Loading Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As work corresponding to item 3 above, if ItemFileIO's API includes Load, you need to implement functions for loading.

When directly inheriting from the ItemFileIO class, implement the following two functions:

* **virtual Item* createItem()**
* **virtual bool load(Item* item, const std::string& filename)**
 
The createItem function should create an instance of the target item type as follows: ::

 Item* FooItem::createItem()
 {
     return new FooItem;
 }

The instance created by this function is used during file loading.

If the target item type is a :ref:`singleton item <plugin-dev-singleton-item-registration>`, you need to return the singleton instance. This can be implemented using ItemFileIO's findSingletonItemInstance function as follows: ::

 Item* FooItem::createItem()
 {
     return findSingletonItemInstance();
 }

When inheriting from the ItemFileIoBase template, the createItem function is implemented by the template, so you don't need to implement it in the derived class.
Also, for the load function, the type of the first argument becomes a pointer to the item type specified in the template parameter.
For example, for FooItem, the definition becomes:

* **virtual bool load(FooItem* item, const std::string& filename)**

In either case, you need to implement the loading process in the load function.
This is implemented similarly to the loader function explained in :ref:`plugin-dev-item-io-function-registration`.
You can use the following ItemFileIO functions:

* **Item* parentItem()**

  * Returns the item that will become the parent after successful loading.

* **int currentInvocationType() const**

  * Returns the type of operation that triggered the loading function call.

  * The value is one of the following defined in ItemFileIO's enumeration type "InvocationType":

  * **Direct**: Direct call from program code

  * **Dialog**: Call from loading dialog

  * **DragAndDrop**: Call by drag & drop operation

  * Direct is set by default.
    
* **std::ostream& os()**

  * Returns the output stream. Output loading messages here.

* **void putWarning(const std::string& message)**

  * Outputs a warning message.

* **void putError(const std::string& message)**

  * Outputs an error message.

Implementing File Saving Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As work corresponding to item 4 above, if ItemFileIO's API includes Save, you need to implement functions for saving.
When directly inheriting from the ItemFileIO class, implement the following function:

* **virtual bool save(Item* item, const std::string& filename)**

As with loading, when inheriting from the ItemFileIoBase template, the type of the first argument of the above function becomes a pointer to the item type specified in the template parameter. In either case, implement it similarly to the save function explained in :ref:`plugin-dev-item-io-function-registration`. ItemFileIO's message output functions can be used in the same way as during loading.

Implementing Input/Output Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to provide additional configuration items (options) for file input/output processed by ItemFileIO, include "Options" in the API.
Then, as work corresponding to item 5 above, implement the following functions that process input/output options:

* **virtual void resetOptions()**

  * Resets options.
    
* **virtual void storeOptions(Mapping* options)**

  * Outputs currently set options to the argument Mapping.
    
* **virtual bool restoreOptions(const Mapping* options)**

  * Inputs options from the argument Mapping.

The option settings for file input/output can be held in any format within ItemFileIO. However, to record the settings in project items and project files, you need to implement mutual conversion with :ref:`plugin-dev-yaml-structured-data-classes` data. This data starts from Mapping, and the above storeOptions and restoreOptions process the conversion between this data and internal state. For restoreOptions, return the success or failure of processing with a bool return value. Also, reset the internal state in resetOptions.

In ItemFileIO's load and save functions, reflect the option settings in the corresponding loading or saving processing.

This establishes the foundation for option support. The remaining question is how to set options in the first place.

Regarding this, you can directly set options for the load and save functions of the `Item class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Item.html>`_. These functions have an options argument as shown in :ref:`plugin-dev-item-file-io-function-program-use` of :doc:`item-file-io`. This options argument actually corresponds to the options mentioned above. When you pass option settings to this argument, they are reflected in ItemFileIO through the above restoreOptions, and then ItemFileIO's load and save functions are executed. This causes file loading or saving to be performed with the settings specified in the options argument.

Options used in the last file loading/saving are recorded in the instance of the `Item class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Item.html>`_. You can also reference the recorded options. These operations can be processed with the following functions of the Item class:

* **void updateFileInformation(const std::string& filename, const std::string& format, Mapping* options = nullptr)**

  * Updates file information that is the target of item loading/saving. Options given in the options argument are recorded in the item.

* **const Mapping* Item::fileOptions() const**

  * Returns options recorded by the above function.

To set options from file dialogs or save them to project files, additional implementation is required. Below we explain these methods.

Implementing Input/Output Option Panels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To set file input/output options from input/output file dialogs, you first need to add "OptionPanelForLoading" or "OptionPanelForSaving" to ItemFileIO's API. These correspond to loading dialogs and saving dialogs respectively. Then, implement the following virtual functions of ItemFileIO for loading and saving respectively:

* Loading

  * **virtual QWidget* getOptionPanelForLoading()**

    * Returns the option panel for loading as a QWidget.
    
  * **virtual void fetchOptionPanelForLoading()**

    * Reflects the current content of the loading option panel to ItemFileIO's option settings.

* Saving

  * **virtual QWidget* getOptionPanelForSaving(Item* item)**

    * Returns the option panel for saving as a QWidget.
    
  * **virtual void fetchOptionPanelForSaving()**

    * Reflects the current content of the saving option panel to ItemFileIO's option settings.

In getOptionPanelForLoading / getOptionPanelForSaving, create a QWidget object that consolidates the configuration GUI to be displayed in the file dialog and return its pointer. That widget will then be inserted into the designated area of the file dialog, allowing users to edit options. There are no particular restrictions on creating the configuration GUI, but please organize it to an appropriate size for display in the file dialog.

In fetchOptionPanelForLoading / fetchOptionPanelForSaving, update the option settings internally managed by ItemFileIO to match the content of the configuration GUI. For example, if you're managing a certain setting with an integer ID and using a combo box for that setting, update the ID variable with the index selected in the combo box. This processing allows option settings made by users in the dialog to be reflected in file loading or saving processing.

The flow of option processing when loading files using a dialog is shown below:      

1. For the ItemFileIO selected in the file dialog's "Files of type" combo, get the loading option panel with the getOptionPanelForLoading function and display it on the dialog.

2. The user operates the option panel to configure options.

3. When the user selects a file and presses the save button, ItemFileIO's fetchOptionPanelForLoading function is executed, and the option panel's settings are reflected in ItemFileIO's option settings.

4. Load the file with the reflected option settings.

5. The option settings used for loading are obtained by ItemFileIO's storeOptions function and recorded in the item with the item's updateFileInformation function.

The processing flow for file saving is as follows:

1. For the ItemFileIO selected in the file dialog's "Files of type" combo, set the item's final options using the restoreOptions function (get final options with the item's fileOptions function). Also, get the saving option panel with ItemFileIO's getOptionPanelForLoading function and display it on the dialog.

2. The user operates the option panel to configure options.

3. When the user selects a file and presses the save button, ItemFileIO's fetchOptionPanelForSaving function is executed, and the option panel's settings are reflected in ItemFileIO's option settings.

4. Save the file with the reflected option settings.

5. The option settings used for saving are obtained by ItemFileIO's storeOptions function and recorded in the item with the item's updateFileInformation function.


Coordination with Project Saving/Loading
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For items that allow file input/output option settings, you need to also record option information in :ref:`plugin-dev-item-file-info-project-save`. This is because when loading files during project restoration, if you don't load files with the same options as when they were last loaded/saved, you cannot make the item content the same.

First, regarding project saving, :ref:`plugin-dev-item-file-info-project-save` explained that you can record file information by implementing the store function as follows: ::

 bool FooItem::store(Archive& archive)
 {
     bool stored = false;
     if(overwrite()){
         if(archive.writeRelocatablePath("file", filePath())){
             archive.write("format", fileFormat());
             stored = true;
         }
     }
     return stored;
 }

Here we were recording file path and file format information. We just need to add option information to this.
And as described above, option information can be obtained with the Item class's fileOptions function.
Using this, we can achieve our goal by modifying the above function as follows:

.. code-block:: cpp
 :emphasize-lines: 7,8,9

 bool FooItem::store(Archive& archive)
 {
     bool stored = false;
     if(overwrite()){
         if(archive.writeRelocatablePath("file", filePath())){
             archive.write("format", fileFormat());
	     if(auto fileOptions = item->fileOptions()){
                 archive.insert(fileOptions);
	     }
             stored = true;
         }
     }
     return stored;
 }

The highlighted parts are additions to the previous code.
If the last used file input/output options are valid, they are written to archive.
The data returned by fileOptions is :ref:`plugin-dev-yaml-structured-data-classes` data, so it can be directly output to archive like this.

And in fact, the recording of option information shown here is processed within the writeFileInformation function shown in :ref:`plugin-dev-item-file-info-project-save`, so it's already achieved with the following code shown there: ::

 bool FooItem::store(Archive& archive)
 {
     bool stored = false;
     if(overwrite()){
         stored = archive.writeFileInformation(this);
     }
     return stored;
 }

For item types that only support file input, the following implementation is sufficient, which is the same: ::

 bool FooItem::store(Archive& archive)
 {
     return archive.writeFileInformation(this);
 }

For the restore function for project restoration, conversely, you need to extract option information from the project file and use it for file loading. The code added to the restore function implementation introduced in :ref:`plugin-dev-item-file-info-project-save` is as follows:
  
.. code-block:: cpp
 :emphasize-lines: 8

 bool FooItem::restore(const Archive& archive)
 {
     bool restored = false;
     string file;
     if(archive.readRelocatablePath("file", file)){
         string format;
         archive.read("format", format);
         restored = load(file, format, archive);
     }
     return restored;
 }
		   
The highlighted line has been modified to specify archive as the third argument of the load function.
This becomes the options argument of the Item class's load function shown in :ref:`plugin-dev-item-file-loading-function`.
If you specify option data here, it will be used during loading.
And since the option data is recorded in archive, you can directly specify archive like this.

And this processing is also included in the loadFileTo function shown in :ref:`plugin-dev-item-file-info-project-save`.
So this too can be written as ::

 bool FooItem::restore(const Archive& archive)
 {
     return archive.loadFileTo(this);
 }

and everything including options will be processed.

Registering ItemFileIO
~~~~~~~~~~~~~~~~~~~~~~

In :ref:`plugin-dev-item-io-function-registration` of :doc:`item-file-io`, we introduced how to register loader and saver functions.
ItemFileIO can also be registered with the system through ItemManager.
Use the following template function of ItemManager: ::

 template <class ItemType>
 ItemManager& addFileIO(ItemFileIO* fileIO);

Specify the target item type in the template argument ItemType.
For fileIO, specify an instance created by new'ing your custom ItemFileIO class.
ItemFileIO is :doc:`referenced`, and its smart pointer is held by ItemManager, so the registering side doesn't need to manage the pointer.

This function is also used from the plugin class's initialize function.
For example, to register FooItemFileIO: ::

  itemManager().addFileIO<FooItem>(new FooItemFileIO);

It's possible to register multiple ItemFileIOs for a certain item type.
It's also possible to additionally register ItemFileIO from another plugin for an item type defined in one plugin, adding file formats supported by the item.