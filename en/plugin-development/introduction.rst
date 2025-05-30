============
Introduction
============

Choreonoid has a :doc:`../basics/plugin` system that allows functionality to be extended or added through software modules called plugins. Plugins are programs written in C++, just like the Choreonoid core, and can implement various processes using the classes provided by Choreonoid.

Many of the features provided by Choreonoid itself are also implemented as plugins. For example, the Body plugin provides basic functionality for handling robot and object models, the PoseSeq plugin provides motion choreography-related functionality, and the Media plugin consolidates media-related functionality. As you can see from this, plugins are not limited to extending specific functionality, but can implement a variety of features.

Of course, users can develop their own plugins. Choreonoid is designed with the premise of extending functionality through plugins, and those who can program in C++ can extend its functionality relatively easily. By developing plugins, you can meet the specific needs of each user and expand the range of applications for Choreonoid in robot research, development, and operation.

This guide explains how to develop new plugins.

Note that by learning how to develop plugins, you will deepen your understanding of Choreonoid's design and implementation, making it easier to contribute to Choreonoid core development and bug fixes. Choreonoid is open source software, and anyone can participate in its development. If you want to improve Choreonoid, please join us in development.