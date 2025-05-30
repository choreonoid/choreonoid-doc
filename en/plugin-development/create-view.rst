============
Creating Views
============

.. contents:: Table of Contents
   :local:

.. highlight:: cpp      

Overview
--------

One of the main components of Choreonoid's GUI is the :ref:`basics_mainwindow_view`.
Views are rectangular areas that can be placed anywhere on the main window and contain interfaces for various display, operation, and editing functions. This section explains how to create and use custom views in plugins.

View Class
----------

All Choreonoid views are implemented as objects of the `View class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1View.html>`_. To create a custom view, you need to define a class that inherits from this View class and implement the view's processing content there.

The View class, like the :ref:`plugin-dev-toolbar-class`, is a widget that inherits from QWidget, allowing you to build any GUI using it as the foundation for the view area. This comes down to how to build displays and interfaces that achieve your objectives through :ref:`plugin-dev-toolbar-use-qt-classes`. Of course, utilizing Choreonoid's diverse components including project items is also important for achieving your goals.

On top of that, the View class provides functions related to the view's state and functionality, and you'll implement views using these as well. Below, we'll introduce the processes required for View implementation by category and explain the related member functions.

.. _plugin-dev-view-initialization:

View Initialization
-------------------

In the View class, you need to build the GUI elements necessary for the view's functionality as initialization processing.
When using Qt's layout classes for this, you can set the top-level layout to the view using the following member functions:

* **void setLayout(QLayout* layout, double marginRatio = 0.0)**

  * Sets the top-level layout for the view.

  * The marginRatio argument sets the margin from the view area boundaries as a ratio of the default margin value. 0.0 means no margin, and 1.0 means the same as the default margin.

*  **void setLayout(QLayout* layout, double leftMarginRatio, double topMarginRatio, double rightMarginRatio, double bottomMarginRatio)**

  * A version of the setLayout function with different arguments. This function allows you to set margins from each boundary (top, bottom, left, right) individually.

These functions add margin adjustment functionality to QWidget's setLayout function.
In views, it's common for a single widget to occupy most of the area, in which case removing margins from boundaries creates a more natural display. These functions are provided to make margin adjustment easier, including such cases.
When specifying 1.0 for marginRatio, the behavior is the same as QWidget class's setLayout function.

You can also set which area on the main window the view should be placed in using the following function.
This is set in the view class constructor:

* **void setDefaultLayoutArea(LayoutArea area)**

  * Specifies the area on the main window where the view is placed by default when first displayed.

  * LayoutArea is an enumeration type defined in the View class, and you specify one of the following values:

    * **TopLeftArea** : Top left
      
    * **MiddleLeftArea** : Middle left
      
    * **BottomLeftArea** : Bottom left
      
    * **TopCenterArea** : Top center
      
    * **CenterArea** : Center
      
    * **BottomCenterArea** : Bottom center
      
    * **TopRightArea** : Top right
      
    * **MiddleRightArea** : Middle right
      
    * **BottomRightArea** : Bottom right

.. note:: The area specified by LayoutArea is guessed from the actually existing view areas on the main window at that time. However, since users can freely change the view layout, areas corresponding to each LayoutArea value don't necessarily exist. Also, the current process for estimating areas is not perfect. Therefore, views may be placed in areas different from what you expect.
	  
Views also have font zoom functionality that can be enabled with the following function:

* **void enableFontSizeZoomKeys(bool on)**

  * Specifying true with this function allows you to zoom the font size used in the view with keyboard operations.

  * The keys used are Ctrl + "+" to zoom in and Ctrl + "-" to zoom out.

.. _plugin-dev-view-state-detection:

Detecting View States
---------------------

Views have "active" and inactive states.
An active state means the view is displayed and can be operated by the user.
Even if a view is included in the main window, it may be stacked with tabs, and views whose tabs are not selected are not visible to users and are not considered active.
Some views perform processing in response to project item changes or user operations, but when a view is not active, the results of processing are not visible to users and the processing is wasted. To avoid this, it's important to perform view processing only when the view is active.

For this purpose, the View class defines the following virtual functions that notify state changes:

* **virtual void onActivated()**

  * Called when the view becomes active.
 
* **virtual void onDeactivated()**

  * Called when the view becomes inactive.

By overriding these functions, you can separate processing for when the view is active and when it's not.
For example, for a view that processes in response to some signal, you can connect to the signal in onActivated and disconnect from the signal in onDeactivated to ensure processing only occurs when the view is active.

Active state changes can also be detected with the following signals provided by the View class:

* **SignalProxy<void()> sigActivated()**

  * Signal emitted when the view becomes active.
 
* **SignalProxy<void()> sigDeactivated()**

  * Signal emitted when the view becomes inactive.

These signals are mainly used when you want to detect view state changes from outside.

Besides active state changes, you can also detect keyboard focus changes for the view area.
This can be achieved by overriding the following virtual function:

* **virtual void onFocusChanged(bool on)**

  * Called when the keyboard focus for the view area changes.

You can check the actual active state and focus of the view with the following member functions:

* **bool isActive() const**

  * Returns true when in active state.

* **bool hasFocus() const**

  * Returns true when keyboard focus is present.


QWidget Event Processing
------------------------

Since the View class inherits from the QWidget class, you can use Qt events notified to QWidget in view implementations.
This makes it possible to perform operations on the view through mouse and keyboard input.
Event detection is basically implemented by overriding the corresponding virtual functions.
For details on actually available events, see the QWidget page in the Qt manual.
Below are some events (corresponding virtual functions) commonly used in view implementations:

* **virtual void keyPressEvent(QKeyEvent* event)**

  * Called when a keyboard key is pressed.

* **virtual void keyReleaseEvent(QKeyEvent* event)**

  * Called when a keyboard key is released.

* **virtual void mouseMoveEvent(QMouseEvent* event)**

  * Called each time the mouse pointer moves over the view.

* **virtual void mousePressEvent(QMouseEvent* event)**

  * Called when a mouse button is pressed.

* **virtual void mouseReleaseEvent(QMouseEvent* event)**

  * Called when a mouse button is released.

* **virtual void mouseDoubleClickEvent(QMouseEvent* event)**

  * Called when a mouse button is double-clicked.

* **virtual void wheelEvent(QWheelEvent* event)**

  * Called when the mouse wheel is operated.

* **virtual void paintEvent(QPaintEvent* event)**

  * Paint request event. Implement this function when drawing directly on the widget.

You can get information about events from the event objects given as arguments to each function.
For example, from the QMouseEvent object given as an argument in mouse-related events, you can get the mouse cursor coordinates and the type of button being pressed. For details on each event, please refer to the Qt manual.

.. _plugin-dev-view-attached-menu:

Implementing Attached Menus
---------------------------

Each view has an attached menu that appears when you right-click on the tab area.
This menu has a default "Detach View" item that allows you to detach the view from the main window.
This attached menu can be customized by adding arbitrary items and can be used as one means of performing view settings and operations.

Menu customization can be achieved by overriding the following virtual function of the View class:

* **virtual void onAttachedMenuRequest(MenuManager& menuManager)**

  * Function called when the attached menu is displayed.

  * You can add menu items through the menuManager argument.


The `MenuManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1MenuManager.html>`_ used as an argument here is a class for managing menus used in the GUI, and you can build arbitrary menus using it.
Qt uses the QMenu and QAction classes to build menus, and MenuManager actually uses these classes internally. Using MenuManager allows you to build menus more efficiently than using Qt classes directly.

.. Create a separate section explaining menus and link to it

Using Widgets Provided by Choreonoid
------------------------------------

As mentioned above, interfaces implemented on views can be freely built using Qt classes.
Qt provides many "widget" classes that serve as GUI components, and you can build various interfaces by combining them.
There are also widgets defined in the Choreonoid SDK that can be used for building views.
Many of these are relatively complex widgets that provide cohesive functionality.
Below we introduce the main widgets defined in the Choreonoid SDK.

First, the following widgets are included in the Base module.
These have general-purpose or foundational functionality:

* `ItemTreeWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemTreeWidget.html>`_

  * Widget that displays project items in tree format.

  * Used in the implementation of the Item Tree View (`ItemTreeView <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemTreeView.html>`_).

  * You can customize the item types to display, the appearance of each item, and context menus. This allows you to create item trees limited to specific tasks.

* `ItemPropertyWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemPropertyWidget.html>`_

  * Widget for displaying and editing project item properties.

  * Used in the implementation of the Item Property View (`ItemPropertyView <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ItemPropertyView.html>`_).

  * You can also customize which properties to display. This makes it easier to use properties for specific purposes.

* `SceneWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1SceneWidget.html>`_

  * Widget that renders scene graphs in 3DCG. Mouse and keyboard operations are also possible.

  * Used in the implementation of the Scene View (`SceneView <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1SceneView.html>`_).

* `PositionWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1PositionWidget.html>`_

  * Widget for displaying and editing the position and orientation of objects (rigid bodies) in 3D space in numerical format.

  * Used in the implementation of the Location View (`LocationView <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1LocationView.html>`_) and `LinkPositionWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1LinkPositionWidget.html>`_ introduced below.

* `GraphWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1GraphWidget.html>`_

  * Widget for displaying trajectory data in graph format.

  * Used in the implementation of various graph display views such as `MultiValueSeqGraphView <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1MultiValueSeqGraphView.html>`_.

The following widgets included in BodyPlugin are also available.
These can be used for manipulating Body models.
By making your custom plugin depend on BodyPlugin, you can use these widgets:

* `LinkDeviceTreeWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1LinkDeviceTreeWidget.html>`_

  * Widget for displaying and selecting links and devices that Body models have in tree or list format.

  * Used in the implementation of the Body plugin's Link/Device View (`LinkDeviceListView <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1LinkDeviceListView.html>`_).

  * Can be used to check the structure of Body models or select links and devices for operation.

* `LinkPositionWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1LinkPositionWidget.html>`_

  * Widget for displaying and editing the position and orientation of links, which are components of Body models, in numerical format.

  * Used in the implementation of the Link Position View (`LinkPositionView <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1LinkPositionView.html>`_).

* `JointDisplacementWidget <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1JointDisplacementWidget.html>`_

  * Widget for displaying and editing joint displacements of Body models using numbers and sliders.

  * Used in the implementation of the Joint Displacement View (`JointDisplacementView <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1JointDisplacementView.html>`_).

For details on the functions of each widget, please refer to the reference manual.

.. _plugin-dev-view-project-save:

View Project Saving
-------------------

Similar to :doc:`item-project-save`, view states can also be saved to project files and restored when loading projects.
This can be achieved by overriding the following virtual functions of the View class:

* **virtual bool storeState(Archive& archive)**

  * Saves the view state.
 
* **virtual bool restoreState(const Archive& archive)**

  * Restores the view state.

These correspond to the :ref:`plugin-dev-state-store-restore-functions` store and restore functions of the Item class, and they also take Archive type arguments. The implementation method is basically the same, so please implement them similarly to :doc:`item-project-save`.

The order in which state restoration functions are called when loading project files is as follows:

1. Each view's restoreState function is called
2. Item restore functions are called in depth-first traversal order of the tree

You need to consider this order when there are state restoration dependencies between views and items.
When referring to view states during item restoration, there's no particular problem since view states are restored first.
However, if item information is needed for view state restoration, implementing it directly in the restoreState function won't work because items haven't been loaded at that point.

This can be resolved using the :ref:`plugin-dev-archive-post-processing` introduced in the :ref:`plugin-dev-archive-class` explanation. Within the view's restoreState function: ::

 archive.addPostProcess([this](){ ... });

By doing this, the lambda expression processing given to the addPostProcess function will be executed after all items have been loaded.

In this post-processing, you can get the actual item from the item's ID value using the findItem function introduced in :ref:`plugin-dev-archive-item-reference` of :ref:`plugin-dev-archive-class`. The ID value for this can be obtained using the Archive class's getItemId function when saving the project. By outputting this value with an appropriate key from the storeState function and saving it to the project file, you can obtain the ID value in the post-processing of restoreState.

View Registration
-----------------

To make implemented views available to users, you need to register the view class with the system.
Use the following function of the `ViewManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ViewManager.html>`_: ::

 template <class ViewType>
 ViewManager& registerClass(
     const std::string& className, const std::string& defaultInstanceName,
     int instantiationFlags = Single);

This is a template function, and you specify the view class to register as the template argument.
The meaning of each argument is shown below:

* **className**

  * Specifies the class name.

* **defaultInstanceName**

  * Specifies the default instance name.

  * The instance name is a name set for each view instance, and this name is displayed as the view title in the tab area.

  * When a view is created by default, the name specified here is used as the instance name. When users manually create views, they also specify the instance name.

* **instantiationFlags**

  * Specifies flags related to view instance creation. Specify a combination of the following flags defined in the ViewManager class:

    * **Single**

      * Only one instance of the view can be created. This flag is the default value for the argument.

    * **Multiple**

      * Multiple instances of the view can be created.
	
    * **Default**

      * One instance of the view is created by default. If this flag is not specified, it is not created by default, and users need to perform creation operations to use the view. However, it is automatically created when loading projects that contain view information.

For instantiationFlags, the default value "Single" is usually fine.
Specify "Multiple" or "Default" as needed.
Note that even when Default is specified and an instance is created by default, the created view is not necessarily displayed in the main window by default. When loading a project, it depends on the view layout information recorded there, and when not loading a project, the default layout built into Choreonoid is used.

View registration is usually done from the plugin class's initialize function, similar to :ref:`plugin-dev-item-type-registration`. Similar to getting ItemManager, you can get the ViewManager instance with the following function defined in the `ExtensionManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ExtensionManager.html>`_, which is the parent class of the `Plugin class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Plugin.html>`_:

* **ViewManager& viewManager()**

For example, to register a view class called FooView: ::

 viewManager().registerClass<FooView>("FooView", "Foo");

Registered views can be placed and used on the main window by checking the corresponding view item in "View" - "Show View" in the main menu. For views that can be created multiple times, you can create and display additional views by selecting the corresponding view item from "View" - "Create View".

.. note:: The `ViewManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ViewManager.html>`_ implements various functions for managing views besides view class registration. Using these, you can also create, get, and display views programmatically. For details on functions used for such processing, please refer to the API reference.