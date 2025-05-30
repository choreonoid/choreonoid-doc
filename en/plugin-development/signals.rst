==============
Using Signals
==============

.. contents:: Table of Contents
   :local:

.. highlight:: cpp

What are Signals
----------------

One of the important concepts in the Choreonoid SDK is signals. A signal is a mechanism for notifying other parts of a program when certain processes are performed or events occur during program execution. Notification specifically means calling some function. You can associate any function that satisfies the signal's type with a signal, and those functions are called when the signal is notified. You can also associate multiple functions with a signal, in which case all functions are called when the signal is notified. This is a type of function callback mechanism, but it has the characteristics of being type-safe and allowing multiple functions to be set. In Choreonoid, we call the functions associated with signals "slots" and the association operation "connection". We will call the process that triggers signal notification "emitting" the signal.

Slot connections to signals can be freely made from anywhere. You don't need to know where in the program the signal is emitted, nor do you need to know what other slots are connected. Conversely, the side emitting the signal doesn't need to know which slots will actually be called. In this way, the signal emitting side and the receiving slots are independent of each other, yet the processes are coordinated through the corresponding signal. This loose coupling allows flexible implementation of various process coordination and can improve the maintainability and extensibility of the entire system.

This signal mechanism is a common technique in software development. This technique corresponds to the `Observer pattern <https://en.wikipedia.org/wiki/Observer_pattern>`_ included in so-called design patterns. There are several names for actual implementations, and the term "signal/slot" became popular with the C++ framework library `Qt <https://www.qt.io/>`_, and has also been adopted by libraries influenced by it such as `libsigc++ <https://libsigcplusplus.github.io/libsigcplusplus/>`_ and `Boost.Signals <https://www.boost.org/doc/libs/1_68_0/doc/html/signals.html>`_. Choreonoid is also influenced by these libraries and uses the same terminology.

Choreonoid also uses this technique to achieve flexible and efficient coordination of various processes. In fact, many signals/slots are implemented in the Choreonoid main body and existing plugins, and it is essential to learn and utilize this mechanism when developing custom plugins.

.. note:: There is also a similar technique called the Publish/Subscribe Pattern, which is incorporated in ROS, a famous middleware for robots. In this pattern, what corresponds to signals is called "messages", and the structure has an even lower degree of coupling between the sender and receiver of messages.

.. note:: Unix-based OSs including Linux have "signals" processed by the kernel. This is a mechanism for providing specific notifications mainly related to OS processes, and should be considered different from Choreonoid SDK signals.


Signal Class
------------

Choreonoid defines its own Signal class to implement the signal mechanism. This is defined in the Util library and can be included with the following description: ::

 #include <cnoid/Signal>

Using the Signal class, you can define objects corresponding to individual signals. A signal object is a kind of function object. The Signal class is defined as a template class and can correspond to any function type.

For example, let's say we use the following function to convey a signal: ::

 void function();

This is a function with no arguments and no return value. In this case, the signal object is defined as follows: ::

 cnoid::Signal<void()> signal;

Since the Signal class is defined within the cnoid namespace of the Choreonoid SDK, when using it outside the cnoid namespace, write cnoid::Signal like this. However, in the following explanation, we will omit the cnoid namespace notation.

This signal is an object corresponding to the type Signal<void()>. You can use this object to connect slots and emit signals.

Let's say the following function is defined on the signal receiver side: ::

 void slot()
 {
     ...
 }

To connect this to the signal, do the following: ::

 signal.connect(slot);

Now slot is connected to signal. Simply pass the function you want to connect to the signal's connect function.

Signal emission is done as follows: ::

 signal();

Since a signal object is a kind of function object, you can write it as if executing it as a function. In reality, when this part is executed, the functions connected to the signal are called.

You can also connect multiple slots. For example, if you have a function: ::

 void slot2()
 {
     ...
 }

Simply connect it again with connect: ::

 signal.connect(slot2);

In this case, when the signal is emitted, both functions slot and slot2 are called. The calling order is the order in which they were connected to the signal.

In this way, you can connect multiple slots to one signal object. In practice, slot connections to signals may be made at various places in the program, and as a result, multiple slots may be connected to a signal without you realizing it. In this case, the part defining the signal, the signal source, and the signal receiver all proceed without specifically identifying where the signal is emitted and which receivers are notified.

I explained that "a signal is a kind of function object", but more precisely, it might be said to be "a function object that can store and call multiple functions together". In fact, the Signal class is implemented almost like this, internally holding multiple function objects in list format.

Note that there may be cases where no slots are connected to a signal object. In that situation, even if you emit a signal, nothing actually happens. Since there are no slots to call, nothing is called and the signal emission process ends. In this case too, the signal source can emit signals without worrying about whether signals are connected. Such cases are actually common.

.. note:: This Signal class is uniquely implemented in Choreonoid, but its design is based on `Boost.Signals <https://www.boost.org/doc/libs/1_68_0/doc/html/signals.html>`_ from the Boost C++ library. Since there are many common parts in usage, that documentation may also be helpful. Note that Boost.Signals has been deprecated since Boost 1.69 and replaced with the successor Boost.Signals2, but that one has become somewhat more complex to use. In fact, Choreonoid used Boost.Signals up to version 1.4, and replacement with Boost.Signals2 was also considered, but as a result of considering ease of handling in Choreonoid's implementation, a unique Signal class was introduced from version 1.5.

.. _plugin-dev-signals-parameters:

Signals with Arguments
----------------------

Signals can also have arguments. For example: ::

 Signal<void(bool on)> boolSignal;

This signal becomes a signal type with one bool type argument.

Then slots connected to this signal need to have this argument. For example: ::

 void boolSlot(bool on)
 {
     ...
 }

This function can be used as: ::

 boolSignal.connect(boolSlot);

And can be used as a slot connected to this signal. Conversely, the previously used: ::

 void slot();

This function cannot be directly connected to this signal because it has different arguments.

With signals that have arguments like this, you can pass arguments when emitting the signal. This is simply: ::

 boolSignal(true);

Just provide arguments to the function call operator on the signal object. This results in: ::

 boolSlot(true);

being executed.

Of course, the value given to the argument can be anything (false here is also fine).

In this way, signals can have any type and any number of arguments. This is specified by giving the function signature (a description of the return value and arguments like a function definition) to the template parameter of the Signal class.

For example, a more complex signal with an int value, a const reference to a std::string object, and a pointer to a Something class as arguments can be realized by writing: ::

 Signal<void(int, const std::string&, Something*)> complexSignal;

Note that you can also write parameter names for argument types in the function signature. In this case, the above definition can be written as: ::

 Signal<void(int value, const std::string& text, Something* something)> complexSignal;

This doesn't affect the signal type itself, but it can be effective for making it easier to understand what each argument means in the definition.

This signal can also be emitted, for example, as: ::

 Something* something = new Something;
 complexSignal(5, "message", something);

Signals Requiring Return Values
-------------------------------

Signals can also require return values. This item is a relatively advanced use of signals, so you can skip it for now. In practice, cases requiring return values are not that common.

A signal requiring a return value can be defined, for example, as follows: ::

 Signal<bool()> rvSignal;

The function signature given to the Signal class indicates that it has a bool type return value. This is the return value that the signal requires. Slots connected to this must be functions that return a bool value accordingly. For example: ::

 bool rvSlot()
 {
     return true;
 }

Such a function.

If you connect this as: ::

 rvSignal.connect(rvSlot);

You can get a bool return value when emitting the signal as: ::

 bool result = rvSignal();

In this example, true is returned and set in the variable result.

This can be used when the signal source wants to know how the receiver processed it. Return values can be of any type other than bool.

However, the return value returned to the signal source may not always be clear. In the above example, if only one slot is connected to the signal, it seems fine if that slot's return value is returned as is. However, if no slots are connected, or if multiple slots are connected, how should the return value be determined? If no slots are connected, there is no value to return. Also, if multiple slots are connected and each slot returns a different value, which value should be returned to the source? These cannot be determined without some rules.

Therefore, there is also a mechanism to solve this for signals requiring return values. This is set with the second argument of the Signal class template parameter. This has a default value, in which case the "value returned by the last called slot" is returned to the signal source. In this case, if no signals are connected, the returned value is undefined.

If you want to change this behavior, specify the second argument of the template parameter. For example: ::

 Signal<bool(), LogicalSum> rvSumSignal;

The returned value becomes the logical sum of the values returned by the slots. That is, if any one slot returns true, it becomes true; otherwise, it becomes false. Another option: ::

 Signal<bool(), LogicalProduct> rvProductSignal;

The returned value becomes the logical product of the values returned by the slots. In this case, if all slots return true, it becomes true, but if any one returns false, it becomes false. As a special situation, if no slots exist, true is returned.

LogicalSum and LogicalProduct given here are objects called Combiners. These are function objects that receive the return values of each slot as iterators and determine the final return value. LogicalSum and LogicalProduct are Combiners predefined in the Signal header. For example, LogicalSum is defined as follows: ::

 class LogicalSum
 {
 public:
     typedef bool result_type;
     template<typename InputIterator>
     bool operator()(InputIterator iter, InputIterator last) const {
         bool result = false;
         while(iter != last){
             if(iter.isReady()){
                 result |= *iter;
             }
             ++iter;
         }
         return result;
     }
 };

The function object's argument InputIterator is an iterator corresponding to each slot's return value, which is looped until the end point last. The key to this implementation is: ::

 result |= *iter;

This ensures that the logical sum of all return values is ultimately returned.

Since any function object with the same format can be set for this part, if you need a different method of determining return values than the default processing, LogicalSum, or LogicalProduct, write and provide a corresponding Combiner yourself.

Using Lambda Expressions
------------------------

Functions connected to signals as slots can be any function that can be called with the same signature as the signal. Therefore, slots don't necessarily need to be statically defined general functions as in the above examples; they can also support various function objects. As one example, it's also possible to use lambda expressions introduced in C++11, which increases the flexibility of slot connections.

As an example of using lambda expressions, first, connection to class member functions (instance functions) becomes possible. For example: ::

 Signal<void()> signal;

And the class: ::

 class A
 {
 public:
     A();
     void functionA();
 };

are defined. Assuming an object of class A is defined as: ::

 A object;

To associate the signal with a call to member function functionA for this object: ::

 signal.connect([&object](){ object.functionA(); });

Or when making similar associations from a function of class A, for example, from the constructor: ::

 A::A()
 {
     signal.connect([this](){ functionA(); });
 }

You can also write it like this.

By using lambda expressions to capture object instances and calling the desired member function within the lambda expression, member functions can also be connected as slots.

In addition to supplementing the hidden argument this of member functions, it's also possible to supplement normal function arguments with lambda expressions.

For example, let's say you want to connect the following function to the above signal: ::

 void functionB(const std::string& text);

In this case, the argument text cannot be obtained from the signal, but if you can determine this string by other means, you can incorporate it into a lambda expression. If a predetermined string is fine: ::

 signal.connect([](){ functionB("Specified Text"); });

You can do something like this, or if you want to specify it with another variable: ::

 string text;

 ...

 signal.connect([text](){ functionB(text); });

It's also possible to do this.

Conversely, you can also connect with functions that don't have arguments included in the signal. For example, for the signal introduced in :ref:`plugin-dev-signals-parameters`: ::

 Signal<void(int value, const std::string& text, Something* something)> complexSignal;

Let's say you want to connect the following function: ::

 void slotWithoutText(int value, Something* something);

This function doesn't have the argument text defined in the signal, so it cannot be connected as is. Even in such cases, you can realize the connection to the signal by making a call that ignores the text value in a lambda expression that also has text as an argument. That is: ::

 complexSignal.connect(
     [](int value, const std::string&, Something* something){
         slotWithoutText(value, something);
     });

.. _plugin-dev-signal-proxy:
     
SignalProxy Class
-----------------

When you include the Signal header, you can also use the SignalProxy class. This generates a proxy object that only allows connection operations for a certain signal object.

For example, let's say a class defines a signal called sigUpdated to convey its state changes. Here's an example showing only the signal part of such a class: ::

 class B
 {
 public:
     Signal<void()> sigUpdated;
 };

Assuming an object of this class is defined as: ::

 B object;

To connect with this signal: ::

 object.sigUpdated.connect(slot);

You can write like this.

However, in this case, since all functions of the signal object can be called, you can emit signals from anywhere: ::

 object.sigUpdate();

However, this signal should originally be emitted when the object's state changes, and it's not something that should be emitted from anywhere. Generally, most signals limit the situations in which they are emitted.

In this case, the problem is that the side connecting the signal can also emit signals. The SignalProxy class is provided to prevent this.

Rewriting the above class B using this becomes: ::

 class B
 {
 public:
     SignalProxy<void()> sigUpdated() { return sigUpdated_; }
 private:
     Signal<void()> sigUpdated_;
 };

SignalProxy is also defined as a template class, and like the Signal class, it takes a function signature as a template parameter. This signature needs to be the same as the target signal. And with the SignalProxy constructor, you can generate a proxy object corresponding to the target Signal object.

In this case, the connecting side can connect functions as: ::

 object.sigUpdated().connect(slot);

The difference from before is that it becomes a member function call to get the SignalProxy, but the connect function can be used the same as before.

In this case, you can connect via SignalProxy, but you cannot emit signals. That is, you cannot do: ::

 object.sigUpdate()();

Since the signal body sigUpdated_ is defined as a private member of class B, the emission of this signal can be managed by the implementation of class B. By introducing SignalProxy in this way, you can separate what can be done between the signal definition source and the connection side. That is, the connection side can only connect, and the emission method can be managed by the signal definition source.

In fact, signals defined in Choreonoid SDK classes return SignalProxy in most cases, and signal emission is done by other means.

.. _plugin-dev-signals-connection-class:

Connection Class
----------------

Once you connect a slot to a signal, that connection is maintained as long as the signal object exists. However, there are times when you want to disconnect the signal and slot connection. The class for doing this is the Connection class. This class becomes available like the Signal class when you include the Signal header.

Actually, the slot connections using the connect function shown so far had a return value. That is an object of the Connection class.

This object can be received by the caller of the connect function. This can be written as follows: ::

 Connection connection = signal.connect(slot);

You can manage the connection with this connection object. To disconnect, just execute the disconnect function as follows: ::

 connection.disconnect();

This disconnects the connection, and after that, the slot function will not be called even if this signal is emitted. In situations where the signal receiver object is destroyed and the slot function cannot be called, you must always disconnect beforehand.

Note that if the signal is destroyed before disconnection, all connections to that signal are automatically disconnected, and the corresponding Connection objects are invalidated. Therefore, if the signal is destroyed first, there's no need to disconnect on the slot side afterward, and even if you do, it will just be ignored without any problems.

Besides disconnection, you can determine whether it's currently connected with: ::

 bool connected = connection.connected();

Another important feature of the Connection class is the connection blocking feature. If you do: ::

 connection.block();

After that, slots will not be called even if signals are emitted. However, in this case, the connection itself is not disconnected. And if you do: ::

 connection.unblock();

Slots will be called again. You can determine whether it's currently blocked with: ::

 bool blocked = connection.isBlocked();

This blocking feature is used in situations where you want to temporarily avoid calling slots.

By the way, with this blocking feature, you need to match block and unblock one-to-one in a relatively short execution range, but if you do: ::

 auto block = connection.scopedBlock();

Block and unblock will be automatically executed at the beginning and end of this block variable's lifetime. This is processed by the constructor and destructor of the Connection::ScopedBlock type object returned by the scopedBlock function.

ConnectionSet Class
-------------------

While Connection was a class for managing a single connection, ConnectionSet is also available as a class for managing multiple connections at once. The usage is almost the same as Connection, but it differs in that it can hold multiple Connections.

For example: ::

 Signal<void()> signalA;
 Signal<void(bool)> signalB;
 Signal<void(int)> signalC;

Let's say there are three signals, and you want to connect corresponding slot functions to use them. However, you want to disconnect all three connections at once when necessary.

You can define three Connection objects as: ::

 Connection connectionA;
 Connection connectionB;
 Connection connectionC;

And connect them as: ::

 connectionA = signalA.connect(slotA);
 connectionB = signalB.connect(slotB);
 connectionC = signalC.connect(slotC);

And when disconnection is needed: ::

 connectionA.disconnect();
 connectionB.disconnect();
 connectionC.disconnect();

This would work, but it becomes a bit cumbersome because you need to manage three Connection objects. It's not uncommon to handle even more signal connections at once.

For this, enable the ConnectionSet class with the following description: ::

 #include <cnoid/ConnectionSet>

Define one ConnectionSet type object as: ::

 ConnectionSet connections;

And when connecting: ::

 connections.add(signalA.connect(slotA));
 connections.add(signalB.connect(slotB));
 connections.add(signalC.connect(slotC));

Then you can disconnect all connections with just: ::

 connections.disconnect();

At this time, each connection is released from the management of connections.

For connection blocking, similar to Connection: ::

 connections.block();

 ...


 connections.unblock();

You can use it like this, and scopedBlock can also be used similarly: ::

 auto block = connections.scopedBlock();

You can determine whether there are currently managed connections with: ::

 bool empty = connections.empty();

And you can get the number of managed connections with: ::

 int n = connections.numConnections();

.. _plugin-dev-signals-scoped-connection:

ScopedConnection and ScopedConnectionSet Classes
-------------------------------------------------

When disconnecting signal connections using Connection or ConnectionSet, you need to explicitly call the disconnect function.
In this case, the programmer needs to be careful about when to disconnect.

On the other hand, there are many cases where you want to match signal connections with a certain processing scope or object lifetime.
In that case, it's reasonable to automatically disconnect according to the processing scope or object lifetime.
The classes for doing this are ScopedConnection and ScopedConnectionSet.
They are Connection and ConnectionSet with added automatic disconnection functionality, respectively.

For example, if you want to receive notifications from signals only during a certain processing scope: ::

 {
     ScopedConnection connection = signal.connect(slot);

     ...

 }

Then at the timing of leaving this scope, that is, when connection is destroyed, the connection between signal and slot is automatically disconnected.

Or if you want to disconnect in accordance with an object's lifetime, define ScopedConnection as a class member variable: ::

 class C
 {
 public:
     C();
     void slot();
 private:
     ScopedConnection connection;
 };


For example, if you connect to a signal in this class's constructor: ::

 C::C()
 {
     connection = signal.connect([this](){ slot(); });
 }

Then if there's a processing flow like: ::

 C* c = new C;

 ...


 delete c;

The signal connection is made when object c is created, and the signal connection is automatically disconnected when c is destroyed.

Even when using the regular Connection class, you could define C's destructor and explicitly disconnect there: ::

 C::~C()
 {
     connection.disconnect();
 }

But with ScopedConnection, you can omit this description, allowing more reliable and efficient disconnection.

Note that as mentioned in :ref:`plugin-dev-signals-connection-class`, the signal might be destroyed first, but there's no problem in that case either. In that case, the connection is already disconnected, and nothing is processed when ScopedConnection is destroyed.

ScopedConnectionSet can manage multiple Connections like ConnectionSet, and in addition, connections are disconnected in the destructor like ScopedConnection. Of course, in this case, all managed connections are disconnected.


Signals Available in Choreonoid SDK
-----------------------------------

We have explained how to use signals above.

Many signals are defined in the Choreonoid SDK. You can check what classes are actually available in the `API Reference Manual <https://choreonoid.org/en/documents/reference/latest/index.html>`_. For example, the `Item class <https://choreonoid.org/en/documents/reference/latest/classcnoid_1_1Item.html>`_, one of the main classes in the Choreonoid SDK, defines 10 signals. Here are excerpts of some signal definitions from them: ::

 SignalProxy<void(const std::string &oldName)> sigNameChanged();
 SignalProxy<void()> sigPositionChanged();
 SignalProxy<void()> sigDisconnectedFromRoot();
 SignalProxy<void(bool on)> sigSelectionChanged();
 SignalProxy<void(bool on)> sigCheckToggled(int checkId=PrimaryCheck);
 SignalProxy<void()> sigUpdated();

While we'll omit the details of each signal here, you can see that they all have the following characteristics:

* Defined as member functions that return SignalProxy

 These are defined for use by the signal connection side. Signal emission is either done in conjunction with object behavior separately or through dedicated functions for signal emission.

* Member function names are in the format sigXXXXX, all with the prefix sig

 All signals defined in the Choreonoid SDK follow this naming convention. The sig prefix indicates that it's a signal.

By understanding these rules, you can determine what signals are available by checking class definitions in the `API Reference Manual <https://choreonoid.org/en/documents/reference/latest/index.html>`_ and other resources. This guide will also introduce practical examples of signal use from here on. Since using signals well in actual plugin development makes it easier to achieve desired functionality, please make sure to utilize them.