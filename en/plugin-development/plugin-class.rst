================
The Plugin Class
================

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

Overview
--------

The Plugin class is the base class used when implementing Choreonoid plugins. It is included in the Base module and defined in the header ``cnoid/Plugin``. A Choreonoid plugin must always be implemented as a custom class that inherits from this class.

As mentioned in :doc:`basics`, the plugin class is the starting point for implementing a plugin. It plays a role similar to that of the main function in an ordinary C program. Basic matters that concern the plugin as a whole, such as registering the plugin name, specifying dependencies on other plugins, and writing initialization and termination processes, are described here.

Note that the Plugin class inherits from the ExtensionManager class. ExtensionManager is a class that provides the basic interface for registering (extending) functions with the Choreonoid main body, and it has access to various manager classes such as ItemManager, ViewManager, and MenuManager. Since the plugin class is also a derived class of ExtensionManager, you can use these features directly. The typical usage is to call ExtensionManager-derived functions through ``this`` from within the plugin class's initialize function to register features.

Class Definition
----------------

The basic form for defining a plugin class is as follows: ::

 #include <cnoid/Plugin>

 class FooPlugin : public cnoid::Plugin
 {
 public:
     FooPlugin();
     virtual bool initialize() override;
     virtual bool finalize() override;
     virtual const char* description() const override;
 };

 CNOID_IMPLEMENT_PLUGIN_ENTRY(FooPlugin)

The class name must end with "Plugin". In the constructor, pass the plugin name to the base Plugin class constructor and set basic information such as dependencies as needed. Among the virtual functions, the initialize function is usually overridden, while the finalize and description functions are overridden as needed.

The ``CNOID_IMPLEMENT_PLUGIN_ENTRY`` macro at the end defines the entry function for retrieving a plugin instance from the shared library. It must appear exactly once in a plugin's source file.

Constructor
-----------

The Plugin class does not have a default constructor; only the following constructor is defined: ::

 Plugin(const std::string& name);

The argument ``name`` is the plugin's name, and the constructor of your custom plugin must always pass a string to this constructor. For example: ::

 FooPlugin::FooPlugin()
     : Plugin("Foo")
 {

 }

As shown, the plugin name is passed using the initializer list.

The plugin name specified here should be the class name with the trailing "Plugin" removed. For the FooPlugin class, the name is "Foo". The plugin name also serves as the identifier referenced by other plugins as a dependency, so choose a name that does not conflict with other plugins.

In the constructor, in addition to setting the plugin name, you specify dependencies (described later) and configure basic plugin attributes. The initialization process itself, however, should be written in the initialize function. At the constructor stage, the initialization of the Choreonoid main body may not yet be complete, so full-fledged processing such as feature registration should be performed in the initialize function.

Overridable Virtual Functions
-----------------------------

The Plugin class defines several virtual functions that can be overridden to customize the plugin's behavior.

The initialize function
~~~~~~~~~~~~~~~~~~~~~~~

::

 virtual bool initialize();

Write the initialization process for the plugin. You will almost always override this function when implementing a plugin class.

This function is called after the plugin is loaded and the constructor has been executed, in an order that takes inter-plugin dependencies into account. Registration of the various features provided by the plugin is normally performed inside this function.

The return value indicates whether the plugin's initialization succeeded. Return true when the initialization process succeeds and the plugin's features become usable. Return false if initialization fails for any reason. When false is returned, the Choreonoid main body does not regard the plugin as usable, and a corresponding message is displayed in the Message View.

The default implementation does nothing and returns true.

The finalize function
~~~~~~~~~~~~~~~~~~~~~

::

 virtual bool finalize();

Write the termination process for the plugin. It is called when Choreonoid terminates.

Override this function when there is processing that must be performed explicitly at termination, such as destroying objects used by the plugin or releasing system resources. The return value indicates whether the termination process succeeded.

Normally, objects managed via ExtensionManager's manage function and various elements registered with the Choreonoid main body are released appropriately by the main body. Therefore, in many plugins there is no need to override the finalize function.

The default implementation does nothing and returns true.

The description function
~~~~~~~~~~~~~~~~~~~~~~~~

::

 virtual const char* description() const;

Returns a description of the plugin. The returned string is referenced by the dialog displayed from the Choreonoid main menu "Help" - "About Plugins".

When publishing a developed plugin externally, override this function and write the plugin's overview, copyright notice, and license conditions here.

Note that the Plugin class provides the following static functions that return the boilerplate text for open-source licenses, which can be used in the implementation of the description function:

* ``static const char* MITLicenseText()`` — returns the boilerplate text of the MIT License
* ``static const char* LGPLtext()`` — returns the boilerplate text of the LGPL

The default implementation returns an empty string.

The customizeApplication function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

 virtual bool customizeApplication(AppCustomizationUtil& app);

A function for customizing the Choreonoid application itself. There is no need to override it in ordinary plugins.

This function is called at an even earlier stage than the initialize function, during the basic initialization of the application. The default implementation returns false.

Specifying Dependencies
-----------------------

When a plugin depends on other plugins, you need to inform the Choreonoid main body of this fact. This ensures that the plugins it depends on are initialized first, and an appropriate error message is displayed if a required plugin is missing.

The require function
~~~~~~~~~~~~~~~~~~~~

::

 void require(const std::string& pluginName);

Specifies the name of a plugin that this plugin depends on. It is called from within the constructor. For example, to depend on the Body plugin: ::

 FooPlugin::FooPlugin()
     : Plugin("Foo")
 {
     require("Body");
 }

When depending on multiple plugins, call the require function once for each dependency.

The precede function
~~~~~~~~~~~~~~~~~~~~

::

 void precede(const std::string& pluginName);

Specifies that this plugin should be initialized before the specified plugin. Unlike the require function, this does not require the specified plugin to exist. It only guarantees that, if the specified plugin exists, this plugin will be initialized before it.

The addOldName function
~~~~~~~~~~~~~~~~~~~~~~~

::

 void addOldName(const std::string& name);

A function for registering previous plugin names so that project files created in the past remain loadable after the plugin name has been changed. It is called from within the constructor. When there are multiple old names, call addOldName for each of them.

Features Inherited from ExtensionManager
----------------------------------------

As mentioned earlier, the Plugin class inherits from the ExtensionManager class. ExtensionManager provides the interface for registering various features with the Choreonoid main body, and these can be used directly from the plugin class. Representative ones are listed below:

* ``ItemManager& itemManager()`` — obtains the ItemManager used to register custom item types, as explained in :doc:`new-item-type`
* ``ViewManager& viewManager()`` — obtains the ViewManager used to register custom view types, as explained in :doc:`create-view`
* ``MenuManager& menuManager()`` — obtains the MenuManager used to add items to the main menu
* ``void addToolBar(ToolBar* toolBar)`` — adds a tool bar to the Choreonoid main body, as explained in :doc:`toolbar`
* ``template<class PointerType> PointerType manage(PointerType pointer)`` — has the lifetime of an object managed by the Choreonoid main body

These functions are normally called from within the initialize function. Since the plugin class is itself an ExtensionManager, they can be used through ``this`` or simply by calling the functions directly.

Defining the Plugin Entry
-------------------------

::

 CNOID_IMPLEMENT_PLUGIN_ENTRY(PluginClassName)

This macro must appear exactly once in the source file that implements the plugin. The argument is the plugin class name.

If this macro is not present, the resulting shared library will not be recognized as a plugin by Choreonoid. Also, only one plugin class can be implemented in a single shared library, so this macro cannot be written more than once.

References
----------

* `API reference for the Plugin class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Plugin.html>`_
* `API reference for the ExtensionManager class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1ExtensionManager.html>`_
