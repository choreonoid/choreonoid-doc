====================
Choreonoid SDK Overview
====================

Overview
--------

C++ Version
-----------

C++11 or later.

Modules
-------

Namespace
---------

Header Files
------------

The actual headers are located under src/Base in the source tree. If you want to know the details of class definitions, please refer directly to those header files. (Note that the actual header files have a .h extension.)


Library Files
-------------

Standard C++ Library
--------------------

Standard C++ library (C++11 or later, also uses C++14 and C++17 functions if available, stdx header)

Dependent Libraries
-------------------

Qt, boost (planned for removal, don't use in new plugins), gettext

Built-in (thirdparty directory)

CMake Variables and Functions
-----------------------------


Reference Manual
----------------

It is possible to generate a reference manual that lists the details of class definitions using a tool called Doxygen, but the commenting for generating explanations is currently insufficient. We plan to improve this in the future. ::

Look at header files and source files.