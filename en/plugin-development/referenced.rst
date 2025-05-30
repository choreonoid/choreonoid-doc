========================
Referenced Type Objects
========================

.. contents::
   :local:

.. highlight:: cpp

Overview
--------

In Choreonoid, many objects created through dynamic memory allocation are defined as classes that inherit from the Referenced class.
Such classes are held with ref_ptr type pointers, which are Choreonoid's own smart pointers.
This pattern is used throughout the Choreonoid SDK, and you will likely use it frequently when creating your own plugins.
Therefore, this section explains the basics of creating and holding such Referenced type objects.

Dynamic Object Creation and Smart Pointers
------------------------------------------

Objects with relatively long lifespans that are shared from multiple parts of a program are typically created with the new operator, placed in dynamically allocated memory, and accessed through pointers. In this case, when the object is no longer needed, it must be deleted with the delete operator to clean up the object and free the memory. However, coding this process without omissions is a burden for programmers, and it often leads to problems such as memory leaks when objects are not deleted.

Therefore, it has become common practice to hold dynamically created objects referenced by pointers using so-called smart pointers. When held by smart pointers, the object is automatically deleted when the pointer variable goes out of scope and the object is no longer referenced from anywhere. C++ provides smart pointers such as std::unique_ptr and std::shared_ptr in its standard library, and by using them, you can more reliably ensure the destruction of dynamically created objects.

Choreonoid's implementation also uses standard library smart pointers, but it also uses its own object types and specialized smart pointers. Therefore, in Choreonoid plugin development, it is necessary to understand the differences between the two and use them appropriately.

Referenced Type Objects and ref_ptr Type Smart Pointers
-------------------------------------------------------

In Choreonoid's implementation, the "own object types and specialized smart pointers" correspond to Referenced types and ref_ptr types, respectively. These implement a mechanism for sharing dynamic objects using reference counters built into the objects.

When using this to hold dynamically created objects, define the object's class by inheriting from the Referenced type. For example, to apply this to a class called Object, you would do the following: ::

  #include <cnoid/Referenced>

  class Object : public cnoid::Referenced
  {
  public:
      void function();

      ...
      
  };

This makes the Object class a Referenced type. This class is typically created with new. Then use ref_ptr type smart pointers to hold the created pointer. This can be written as follows: ::

  cnoid::ref_ptr<Object> object = new Object;

The object can be handled like a regular pointer (raw pointer). For example, member functions of the referenced object can be executed as follows: ::

  object->function();
  
Since writing ref_ptr every time makes the description somewhat complex, it is common to define it in advance as a pointer type specific to this class using typedef. In this case: ::

  typedef ref_ptr<Object> ObjectPtr;

and then typedef it, so you can write: ::

  ObjectPtr object = new Object;

When typedef'ing a type name like this, use the naming convention "ClassName + Ptr". By following this rule, you can recognize that the typedef'd type name is a smart pointer type.

Since it's a smart pointer, the referenced object is also destroyed when this pointer is destroyed. Therefore, you don't need to explicitly execute delete on the object, and you shouldn't. For example: ::

  {
      ObjectPtr object = new Object;

      ...

  }

In this case, object is also deleted when this scope ends.

Of course, actual use cases rarely involve such simple scopes. In practice, this pointer is defined as a member variable of other objects and referenced from there, or passed to multiple objects. References by ref_ptr allow multiple pointers to redundantly reference the same object. In that case, the object is destroyed when it is no longer referenced by any pointer.

As you can see from the above, this is basically the same as std::shared_ptr, as it is a smart pointer that allows multiple pointers to share and reference an object. The method to achieve this is also basically the same, as both use reference counters.


Differences Between shared_ptr and ref_ptr
------------------------------------------

The purpose of referencing classes defined with Referenced type using ref_ptr is basically the same as using std::shared_ptr.
So what is the difference between the two? The ref_ptr type has higher compatibility with raw pointers compared to shared_ptr.
This results in slightly different coding styles between the two.

First, when creating objects, with shared_ptr, rather than: ::

  shared_ptr<Object> object = new Object;

it is slightly more efficient to write: ::

  shared_ptr<Object> object = make_shared<Object>();

so this notation is basically used. With Referenced + ref_ptr, it is always the former notation.

.. note:: For cases where dynamic creation is assumed, there is also a style where, for example, a static member function called create is defined: ::

       shared_ptr<Object> object = Object::create();

   to limit the creation method. In this case, there is no particular difference between shared_ptr and ref_ptr.

Also, with shared_ptr, the assumption is to use shared_ptr in all situations when using it. For example: ::

  shared_ptr<Object> object = make_shared<Object>();
  ...
  
  Object* object2 = object.get();
  ...
  
  shared_ptr<Object> object3 = object2;

This is not possible. Regenerating a shared_ptr from the raw pointer object2 would conflict with the reference management of the original shared_ptr.

On the other hand, with Referenced + ref_ptr, it is not a problem to go through raw pointers in the middle like this. Only pointers that continuously hold references to objects need to be ref_ptr, and for temporary references such as function arguments, it is OK to use raw pointers. In fact, there are many places in Choreonoid's implementation where ref_ptr and raw pointers are used separately in this way.

.. note:: Regarding this, by introducing enable_shared_from_this, it seems that similar usage can be achieved with shared_ptr as well. However, this is not typically done actively...

In summary, Referenced + ref_ptr is easier to write in combination with raw pointers, and this is the biggest difference from shared_ptr. This characteristic can make code description slightly more concise in some cases. Also, in such cases, the overhead related to reference counters is reduced, so performance may be slightly better depending on the usage situation.

These differences come from whether the reference counter is allocated outside the object or held by the object itself. shared_ptr uses the former method, while Referenced + ref_ptr uses the reference counter defined as a member variable of the Referenced class.

Smart pointers using Referenced + ref_ptr were introduced through trial and error in the early stages of Choreonoid development and have continued to be used since then. Please note that their usage is slightly different from shared_ptr, which is the C++ standard smart pointer.

   
weak_ref_ptr
------------

For shared_ptr, weak_ptr is available to hold weak references to target objects. Similarly, weak_ref_ptr is available for holding weak references for ref_ptr. The usage is almost the same as weak_ptr. It can be created from ref_ptr as follows: ::

 ref_ptr<Object> object = new Object;
 weak_ref_ptr<Object> wobject = object;

Like ref_ptr, it can also be created from raw pointers: ::

 weak_ref_ptr<Object> wobject = new Object;

Like weak_ptr, you can create ref_ptr with the lock function: ::

 if(ref_ptr<Object> obj = wobject.lock()){
     ...
 }

Other functions such as reset and expired can also be used in the same way as weak_ptr.


Making Custom Classes Referenced
--------------------------------

When defining custom classes in plugin implementation, it is also possible to inherit from Referenced type and reference them with ref_ptr.
When inheriting from existing classes that are Referenced types, newly defined classes will inevitably also be Referenced types, but otherwise it is not necessarily required to make them Referenced types.
Classes that are intended to be dynamically created using new and that you want to reference with smart pointers are candidates for Referenced types.
Even if there is a possibility of using them in this way, if there is also a possibility of using them directly as automatic variables or class member variables, they should not be made Referenced types. Such classes should be referenced with shared_ptr only when they are dynamically created and used.