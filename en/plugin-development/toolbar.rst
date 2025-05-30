================
Creating Toolbars
================

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

Overview
--------

One of the GUI elements that can be added by plugins is a "toolbar".
This is relatively easy to create and can be used as an interface for using implemented functions.
Here we explain an overview of how to create them.

.. _plugin-dev-toolbar-class:

ToolBar Class
-------------

Toolbars are created using the `ToolBar class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ToolBar.html>`_ from the Choreonoid SDK. ToolBar is the base class for toolbars. It inherits from Qt's QWidget and reserves an area for toolbar use as the minimum unit in the OS window system. You can arrange various GUI components such as buttons on top of this. However, the types of GUI components to arrange on toolbars are somewhat predetermined, and the arrangement must follow the toolbar style. The ToolBar class provides functions for efficiently constructing toolbars with these considerations in mind.

There are two methods for actually creating toolbars:

A. Create an instance of the ToolBar class and add necessary interfaces from outside
B. Define your own class that inherits from the ToolBar class and manage necessary interfaces inside the class

Method A is simpler, so if your implementation can be easily done this way, you should adopt this method. Simple toolbars are better suited to this approach.

In this case, first create it like: ::

 ToolBar* toolBar = new ToolBar("MyToolBar");

As shown, toolbars are usually created dynamically using the new operator. You must also provide a name for the toolbar in the constructor. This name is used in menus for toggling toolbar display and in toolbar information saved in project files.

For the created toolbar object, construct the toolbar by doing: ::

 toolBar->addButton( ... );
 ...

This is processed in the plugin's initialize function, etc. The created toolbar will be registered later.

Method B is suitable when the toolbar interface becomes complex or when implementing various information and processing within the toolbar. In this case: ::

 class MyToolBar : public ToolBar
 {
 public:
     MyToolBar() : ToolBar("MyToolBar")
     {
         addButton( ... );
         ...
     }
 };

In this case as well, create an object of the created class and perform the registration process.

In any case, creating a toolbar consists of the following work elements:

1. Construct interfaces such as buttons
2. Implement various state management and processing for function realization as needed
3. Coordinate interfaces with various processes

The ToolBar class API is mainly provided to support 1. Also, for 1, you will utilize the Qt library that Choreonoid uses for GUI implementation.

2 and 3 are not particularly limited to toolbars and depend on the functions to be realized.
However, for 3, coordination is often done using signals.

This section mainly explains the overview of 1 and the signals used to implement 3. For specific examples related to 1-3, we will introduce samples in the next section.

.. _plugin-dev-toolbar-functions:

Functions for Interface Construction
------------------------------------

The `ToolBar class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ToolBar.html>`_ provides basic functions for constructing toolbar interfaces. Here we introduce some of the main functions.

First, buttons are GUI components often implemented in toolbars. You can add buttons with the following functions of the ToolBar class:

* **ToolButton* addButton(const QString& text, const QString& tooltip = QString())**

  Adds a button to the toolbar. Set the button text with text. If you specify tooltip, this text will be displayed as a tooltip when the mouse cursor is held still over the button.

* **ToolButton* addButton(const QIcon& icon, const QString& tooltip = QString())**

  Adds a button similarly to above, but this creates a button with an icon instead of text. Specify the icon image with icon.

These buttons are ordinary buttons that perform some action when pressed. These are also called push buttons.

Since buttons are actually created as Qt objects, the arguments given to the above functions are currently Qt classes. Strings are QString type instead of std::string, and icon images are specified with QIcon type objects.

For details on each Qt class, please refer to the Qt manual. To briefly introduce how to use QString, first, string literals can be passed directly to this type. Therefore: ::

 toolBar->addButton("Button");

can be set like this. If you want to set a std::string string, set it by converting to C language string format (const char*) with the c_str function: ::

 std::string text("Button");
 toolBar->addButton(text.c_str());

For QIcon to set icon images, you can create them from image files. For example, if you prepare an SVG file called "icon.svg" for an icon: ::

 QIcon icon("icon.svg");

You can create a QIcon object of that image. (Of course, specify the actual file path appropriately.)

.. note:: Qt has a resource system that allows embedding arbitrary files into executable or shared library binaries. Embedded files can be read from programs just like regular files. Icon images should actually be embedded in binaries using this resource system. For details, please refer to the Qt manual.

Other types of buttons are also available in toolbars. Another commonly used type is the toggle button. This has a pressed state and a non-pressed state, used like a switch to toggle on/off. This can be added with the following functions. The meaning of each argument is the same as addButton.

* **ToolButton* addToggleButton(const QString& text, const QString& tooltip = QString())**
* **ToolButton* addToggleButton(const QIcon& icon, const QString& tooltip = QString())**

Radio buttons can also be used. Radio buttons are an interface for choosing from several options. For example, you prepare 3 radio buttons and allow only one of them to be on (pressed). When one is turned on, the remaining buttons turn off, allowing the user to communicate their selection. These can be added with the following functions:

* **ToolButton* addRadioButton(const QString& text, const QString& tooltip = QString())**
* **ToolButton* addRadioButton(const QIcon& icon, const QString& tooltip=QString())**
  
Note that radio buttons need to be created for each group of choices. If there are two or more groups, execute the following function before adding buttons belonging to a new group:

* **void requestNewRadioGroup()**

Other elements besides buttons can also be added:

* **QLabel* addLabel(const QString& text)**

  Adds a text label. The specified text is displayed on the toolbar.

* **QWidget* addSeparator()**

  Adds a separator. A vertical bar is displayed to separate GUI elements added before and after this.

* **void addSpacing(int spacing = -1)**

  Adds spacing. Separates GUI elements added before and after by the set amount. Using the default argument results in standard width spacing.

First, construct the toolbar interface using such functions.

Button Objects
--------------

Adding buttons to a toolbar alone is not meaningful. You need to execute related processing when buttons are pressed or change processing content according to button states. To do this, first obtain each button's object, then get states from it and make signal connections.

Button objects can be obtained as return values from the above button addition functions.
These are pointers to objects of the `ToolButton class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ToolButton.html>`_. The ToolButton class inherits from Qt's QToolButton class and makes some QToolButton signals available as Choreonoid-format signals.

For example: ::

 ToolButton* button = toolBar->addButton("Button");

You can obtain the ToolButton object corresponding to the added button. The following signals are available for this ToolButton class:

* **SignalProxy<void()> sigClicked()**

  Emitted when the button is pressed.
 
* **SignalProxy<void(bool on)> sigToggled()**

  Emitted when the on/off state of a toggle button changes. The state after the change is given by the argument on.

.. note:: These signals have corresponding Qt signals. The corresponding signals are defined in QAbstractButton, the parent class of QToolButton, and are signals called clicked and toggled respectively. You can use those Qt signals directly, but by using Choreonoid-format signals, you can improve the consistency of the entire code.

For example, for the variable button in the previous code: ::

  button->sigClicked().connect([](){ onButtonClicked(); });

Then the onButtonClicked function is called when this button is pressed. For toggle buttons or radio buttons: ::

  button->sigToggled().connect([](bool on){ onButtonToggled(on); });

Then the onButtonToggled(on) function is called when the button state changes.

In this way, you execute processing related to button operations in the connected functions.

Also, for toggle buttons and radio buttons, you can get the button's on/off state with the following function:

* **bool isChecked() const**

The obtained pointer to the button object is valid while the button exists. Unless you perform processing to delete buttons or toolbars, the created toolbar and buttons added to it usually exist until the application ends. Therefore, you can hold the obtained pointer and reference it later when related operations are executed.

.. note:: Qt classes inherited by ToolButton define many functions besides isChecked, and all of them can be used. QToolButton inherited by ToolButton further inherits QAbstractButton, and the Qt class hierarchy continues with QWidget and QObject beyond that, and all functions included in all these classes are available. Please check the Qt manual for functions that can actually be used.

.. _plugin-dev-toolbar-use-qt-classes:
 
Utilizing Qt Classes
--------------------

You can add any Qt widget to a toolbar using the following function of the ToolBar class:

* **void addWidget(QWidget* widget)**

A widget represents individual GUI components and is a term commonly used in GUI libraries. In Qt's case, the QWidget class is the base class for widgets, which corresponds to the minimum unit of drawing area managed by the window system. Various widgets derived from this foundation are available, and QToolButton introduced above is one such widget.

Basically, any widget defined in Qt can be added to a toolbar with the above function. For example, Choreonoid's main time bar includes a spin box for displaying and entering time numerically and a slider for visually displaying and changing time, which are implemented with Qt's QDoubleSpinBox and QSlider widgets respectively. However, since toolbars are designed as bars arranged horizontally, it's better not to forcibly add widgets that don't fit this form.
  
You can see implementations of various toolbars including TimeBar in Choreonoid's main source code, so please refer to them when creating your own toolbars.

.. note:: Since Choreonoid uses Qt for GUI implementation, Qt knowledge is essential when implementing GUI-related parts in plugins. Conversely, since Choreonoid's GUI is almost entirely Qt, if you understand Qt, you should be able to realize most GUIs. In Choreonoid, besides toolbars, views can also be added by plugins, which will be explained later in this guide. There too, if you follow certain rules for implementing views, the rest becomes a matter of how to use Qt classes to realize the desired GUI.

.. When creating custom views, introduce classes that extend Qt with Choreonoid and include a list. The main purpose is to enable direct use of Choreonoid-format signals. I don't think we need to mention this in this section yet.

Registering Toolbars
--------------------

Toolbar creation is usually done in the initialize function of your custom plugin class. After creating a toolbar, register it with the system using the addToolBar function. This process looks like: ::

 class ToolBarPlugin : public Plugin
 {
 public:
     ...
      
     virtual bool initialize() override
     {
         auto toolBar = new ToolBar("MyToolBar");
         auto button = toolBar->addButton("Button");
 
         ...
         
         // Register the toolbar
         addToolBar(toolBar);
 
         return true;
     }
 }

The addToolBar function used here is defined in the `ExtensionManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ExtensionManager.html>`_. Since the `Plugin class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Plugin.html>`_ inherits from ExtensionManager, you can use this function.

Registered toolbars are displayed in the toolbar area. If you don't want them displayed by default: ::

  toolBar->setVisibleByDefault(false);

Even in this case, make sure to register it. For registered toolbars, you can toggle display on/off from the main menu "View" - "Show Toolbars". Even if not displayed by default, it's possible to switch to display from there.