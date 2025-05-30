============================
Signal Usage Sample (S02)
============================

.. contents:: Table of Contents
   :local:

Overview
--------

In this section, we present a sample plugin that uses signals and explain the practical usage of signals. The sample introduced here is a slight modification of :doc:`minimum-sample` that outputs messages in conjunction with time bar operations.

Source Code
-----------

.. highlight:: cpp

Create a plugin source directory as in :doc:`minimum-sample` and create the following source code with the filename DevGuidePlugin.cpp: ::

 #include <cnoid/Plugin>
 #include <cnoid/MessageView>
 #include <cnoid/TimeBar>
 #include <fmt/format.h>
 
 using namespace cnoid;
 
 class DevGuidePlugin : public Plugin
 {
     ScopedConnection connection;
 
 public:
     DevGuidePlugin() : Plugin("DevGuide")
     {
 
     }
 
     virtual bool initialize() override
     {
         connection =
             TimeBar::instance()->sigTimeChanged().connect(
                 [this](double time){ return onTimeChanged(time); });
         return true;
     }
 
     bool onTimeChanged(double time)
     {
         MessageView::instance()->putln(fmt::format("Current time is {}", time));
         return true;
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)

The CMakeLists.txt for building can use the same description as :doc:`minimum-sample`.

Plugin Execution
----------------

When you run Choreonoid with this plugin installed, the onTimeChanged function of this plugin will be called in conjunction with time operations through the time bar. As a result, when you operate the time bar slider or press the play button, the current time will be displayed as text in the message view as the time changes in Choreonoid. Try operating the time bar to verify the behavior.

Source Code Explanation
-----------------------

We'll explain the newly introduced parts in this sample. (We'll omit explanations for parts that are the same as :doc:`minimum-sample`.) ::

 #include <cnoid/TimeBar>

This sample uses time bar signals. Therefore, we include the header for the `TimeBar class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1TimeBar.html>`_. ::

 #include <fmt/format.h>

We include the header for the fmt library used for text output to the message view. ::

 class DevGuidePlugin : public Plugin
 {
     ScopedConnection connection;

     ...


We define a ScopedConnection object as a member variable for disconnecting signals and slots. This is used to automatically disconnect the signal connection made by this plugin when the plugin is released. ::

 virtual bool initialize() override
 {
     connection =
         TimeBar::instance()->sigTimeChanged().connect(
             [this](double time){ return onTimeChanged(time); });
     return true;
 }

We connect signals and slots in the plugin initialization function. The target signal is TimeBar's "sigTimeChanged" signal. This is defined in the TimeBar class as follows: ::

 SignalProxy<bool(double time), LogicalSum> sigTimeChanged();

As a function type, it has one double type argument and returns a bool value as a return value. When multiple slots are connected, the return value to the signal emitting side is determined by LogicalSum. That is, if any one returns true, it becomes true. And since it's defined as a member function that returns SignalProxy, only connection is possible. Note that ::

 TimeBar::instance()

is a static member function that returns the only instance of the TimeBar class as a TimeBar pointer. There are many classes in Choreonoid that return instances in this singleton pattern, and they can be used in the same format.

The body of the slot is the member function onTimeChanged of DevGuidePlugin. Since this is a member function, we use the following lambda expression for connection with the signal: ::
 
 [this](double time){ return onTimeChanged(time); });

This lambda expression supplements the instance variable this for calling the plugin's member function. This is equivalent to: ::
 
 [this](double time){ return this->onTimeChanged(time); });

And onTimeChanged is called on the object of this pointer like this. This writing style might be clearer.

Regarding the sigTimeChanged signal, as can be guessed from the name, it's a signal emitted when the time managed by the time bar changes. This is defined in the `TimeBar class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1TimeBar.html>`_ as follows: ::

 SignalProxy<bool(double time), LogicalSum> sigTimeChanged();

This is a signal emitted in all situations where the time bar time changes, whether entering a value in the time display spin box, operating the slider, or performing animation with the play button. During animation playback, it's emitted periodically at the interval of the time bar's playback frame rate. The argument time is the current time. Also, it's a signal that requires a return value from slots, and you must return true if valid processing was performed at that time, and false otherwise. This return value is particularly referenced during animation. By referencing the LogicalSum of all slot return values, if any one is true, the animation continues, and if all are false, it stops. In other words, if there's valid processing at each time during animation, the animation continues, but it stops when that's no longer the case.

Note that by doing: ::

 connection =
     ...

We assign the Connection object returned by the signal's connect function to the member variable connection. As mentioned above, since connection is of ScopedConnection type, this connection is also disconnected when the plugin is released (deleted). Since plugins are usually deleted when the application ends, there's actually no particular problem even without this processing in this case. That said, it's desirable to disconnect the connection to a slot when that slot can no longer be called, so it's better to include such processing when connecting member functions as slots. Here we've written it this way, also serving as an example of using Connection objects. ::

 bool onTimeChanged(double time)
 {
     MessageView::instance()->putln(fmt::format("Current time is {}", time));
     return true;
 }

This is the body of the slot connected to TimeBar's sigTimeChanged signal. It outputs a message of the current time to the message view. Since it returns true as a return value, the animation always continues.

Here we use the `text formatting library fmt <https://github.com/fmtlib/fmt>`_ to create the message string. You can think of this library's format function as the C++ version of C's printf function. The {} in the text is called a "replacement field" and is replaced with the value of time specified as the second argument of the format function. Since this message is simple, we might not necessarily need to use this, but we're using it to introduce the library. The fmt library is a well-regarded text formatting library for C++, and it has been incorporated into the standard C++ library as `std::format <https://cpprefjp.github.io/reference/format/format.html>`_ since C++20. It's frequently used in Choreonoid's source code as well, so if you're not familiar with it, it would be good to learn it on this occasion.