How to Find the Location of a Crash Using Core Files
====================================================

When receiving reports of Choreonoid crashes, there are cases where proprietary plugins or controllers are being used that cannot be shared, making it impossible for Choreonoid developers to reproduce the situation. In such cases, while it is fundamentally difficult to identify or fix the cause of the crash, having information about where in the program the crash occurred might help provide some clues.

On Linux, information about where a program crashed can be obtained using a mechanism called Core files (Core dumps). Here we introduce an overview of this mechanism.

.. contents:: 
   :local:
   :depth: 1


Enabling Core Dumps
-------------------

A Core file is a file named "core" that is output to the current directory when a program crashes. This file contains information about the program's execution, and by loading it into a debugger, it is possible to analyze to some extent the process that led to the program crash. The output of Core files is called a "core dump", and in Japanese, expressions like "spitting core" are also used. For more details, searching for terms like "Core file" or "core dump" will yield various results, so please refer to those.

The advantage of Core files is that you can get hints about where the crash occurred without any special preparation or operations for debugging the target program. You can run a normally built program in the normal way, and when it crashes, you can obtain some information on the spot.

Here we briefly introduce the procedure for enabling core dumps on Ubuntu Linux. On Ubuntu Linux, the default setting is such that Core files are not output even when a program crashes. To enable output, in the file /etc/default/apport, set::

 enabled=0

(There are other methods for configuring Core output, but we omit the details here.)

After switching to this setting, restart Ubuntu to reflect the configuration.

Then, as a shell setting, you need to execute the following command::

 ulimit -c unlimited

If it's tedious to execute this every time, you can write it in your home .profile file or similar.

.. note:: On Ubuntu 20.04 and later, writing the above command in .profile doesn't seem to enable Core file output. In this case, we have confirmed that writing it in .bashrc instead makes it effective.

With the above settings, when a program crashes, a file named "core" will be output to the current directory.

Note that core files from previous outputs may remain. You can check whether it's a file output at that time by looking at the timestamp of the core file. If the timestamp matches the expected crash time, you can safely assume it's the core file output at that point.

Loading into GDB
----------------

By loading a Core file into the debugger "GDB", you can check the information stored in it. In this case, start GDB as follows::

 gdb executable corefile

For the "executable" part, specify the executable file of the program that was running when it crashed. Make sure to include the path to the file so that the executable can be identified.

For the corefile part, specify the corresponding core file (usually "core").

For example, if you run the Choreonoid executable generated under the build directory "build" in the Choreonoid source directory, and then Choreonoid crashes and outputs a core file, you would do the following in that source directory::

 gdb ./build/bin/choreonoid core

Displaying the Backtrace (Call History)
---------------------------------------

When successfully loaded into GDB, the GDB command prompt is displayed as follows::

 (gdb)

You can perform various operations by entering GDB commands here.

What we introduce here is the backtrace command. This refers to the stack information stored in the Core file and displays the function call history up to the crash location.

To execute this command, type backtrace or bt at the GDB command prompt and press the return key::

 (gdb) bt

This will display text like the following on the screen::

 #0  0x00007f93cd754078 in cnoid::Item::insertChildItem(cnoid::Item*, cnoid::Item*, bool) ()
   from /home/nakaoka/choreonoid/build/lib/libCnoidBase.so.1.8
 #1  0x00007f93cd74cec9 in cnoid::ItemFileIO::Impl::loadItem(cnoid::ItemFileIO::InvocationType, cnoid::Item*, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, cnoid::Item*, bool, cnoid::Item*, cnoid::Mapping const*) () from /home/nakaoka/choreonoid/build/lib/libCnoidBase.so.1.8
 #2  0x00007f93cd74d1a2 in cnoid::ItemFileIO::loadItem(cnoid::Item*, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, cnoid::Item*, bool, cnoid::Item*, cnoid::Mapping const*) ()
   from /home/nakaoka/choreonoid/build/lib/libCnoidBase.so.1.8
 #3  0x00007f93cd74722f in cnoid::ItemManager::load(cnoid::Item*, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, cnoid::Item*, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, cnoid::Mapping const*) () from /home/nakaoka/choreonoid/build/lib/libCnoidBase.so.1.8

This displays as many lines as there are function calls in the history. The deeper the function calls up to the crash point, the more text (lines) will be displayed. When it cannot be displayed on the screen at once, the following message appears::

 ---Type <return> to continue, or q <return> to quit---

You can press return here to display the continuation.

In this backtrace, the function displayed at the top with #0 is the function where the crash occurred. In the above example, the insertChildItem function of Choreonoid's Item class is displayed, indicating that the crash ultimately occurred in this function. By looking at the subsequent lines, you can also see what function calls led to this function.

Note that when a program is built using normal build methods, optimization is applied during source code compilation. In that case, due to reasons such as function calls being inline-expanded, it may not accurately reflect the actual function where the crash occurred in the source code. However, even in that case, some function that called the crashed function will be displayed, providing more clues than having no information at all.

When reporting Choreonoid crashes, providing this information can help developers get clues about what the problem might be, so we encourage you to make use of it. (Please note that providing this information on forums or elsewhere does not guarantee that Choreonoid developers will provide a solution.)