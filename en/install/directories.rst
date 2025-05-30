Directory Structure
==================

Choreonoid is generally installed with the following directory structure: ::

 + Installation destination
   + bin
   + lib
     + choreonoid-x.x
   + share
     + choreonoid-x.x
       + project
       + model

For Windows, it looks like this: ::

 + Installation destination
   + bin
   + lib
     + choreonoid-x.x
   + share
     + project
     + model


Here, the "x.x" part of "choreonoid-x.x" represents the version number of Choreonoid, which would actually be a number like "1.6".

The contents of each directory are as follows:

**bin**
 Contains executable files.

**lib**
 Contains library files.

**lib/choreonoid-x.x**
 Contains plugin files. Also, if there are other files related to system execution, they are placed in subfolders according to their type. This directory is called the **"plugin directory"**.

**share/choreonoid-x.x or share**
 Contains various data files such as documentation and samples. This directory is called the **"share directory"**. The actual files are placed in subfolders such as "project" and "model" according to their type.


Note that the share directory also exists in the source archive with the same folder structure, containing sample data and other files, so you can use those when working with samples. Also, when building from source code on Linux, bin and lib directories are generated within the build directory, and the binary files generated there are also executable.