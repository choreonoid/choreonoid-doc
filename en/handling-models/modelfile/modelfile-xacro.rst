xacro (Macros for URDF)
=======================

.. contents::
   :local:
   :depth: 3

Overview
--------

xacro (XML macro) is a macro language for XML. Using xacro allows you to write XML in a shorter and more readable way. The robot model notation `URDF (Unified Robot Description Format) <modelfile-urdf.html>`_ available in Choreonoid uses XML. Therefore, xacro contributes to simplifying the description of robot models.

The latest version of Choreonoid bundles xacro, so you can use xacro functionality regardless of whether you have a ROS environment or not (currently only supported on Linux).

Note that in xacro files, you need to add the attribute xmlns:xacro="http://www.ros.org/wiki/xacro" to the robot tag.

The following is a brief introduction to xacro functionality based on the `official xacro documentation <http://wiki.ros.org/xacro>`_.

.. _xacro-file-reference-properties:

Properties
----------

The property macro serves as a variable within XML documents. There are two ways to use the property macro.

The first is to use numbers or strings as variables. First, define a variable with the xacro:property tag using the variable name (name attribute) and value (value attribute). Then, use the variable where needed with the notation called dollared-braces: ${variable_name}.

In the following example, a cylindrical geometry is specified through two variables, radius and length.

Example::

    <xacro:property name="radius" value="2.1"/>
    <xacro:property name="length" value="4.5"/>

    <geometry type="cylinder" radius="${radius}" length="${length}"/>

The second method is called property blocks. In property blocks, instead of specifying a value, you can treat the part enclosed by the xacro:property tag as a variable. However, when using this, you must use the xacro:insert_block tag.

In the following example, the predefined origin tag is applied using the xacro:insert_block tag. Property blocks are useful when you want to use tags with the same content in various places.

Example::

    <xacro:property name="link_origin">
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
    </xacro:property>

    <link name="sample_link">
      <inertial>
        <xacro:insert_block name="link_origin"/>
        ...
      </inertial>
    </link>

Dictionaries, Lists, and YAML Files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In xacro, you can use Python dictionary format and lists when defining variables.

Example (Python notation)::

    <xacro:property name="props" value="${dict(a=1, b=2, c=3)}"/>
    <xacro:property name="props_alt" value="${dict([('1a',1), ('2b',2), ('3c',3)])}"/>
    <xacro:property name="numbers" value="${[1,2,3,4]}"/>

You can also load YAML files and define variables directly from YAML files. At this time, all variables in the YAML file are stored in a dictionary.

Example (loading YAML file)::

    <xacro:property name="yaml_file" value="$(find package)/config/props.yaml" />
    <xacro:property name="props" value="${load_yaml(yaml_file)}"/>

YAML file::

    a: 1
    b: 2
    c: 3

Example of using variables in a dictionary::

    <xacro:property name="a" value="${props['a']}"/>

.. _xacro-file-reference-mathematical-expression:

Mathematical Expressions
------------------------

In xacro, you can also perform calculations within XML. You can use arithmetic operations, comparisons, constants defined in Python's math module (such as pi), and functions (such as trigonometric functions) written inside dollared-braces (${}).

Example::

    <xacro:property name="R" value="2"/>
    <xacro:property name="alpha" value="${sin(30/180*pi)}"/>

    <limit lower="${radians(-90)}" upper="${radians(90)}" effort="0" velocity="${radians(75)}"/>

.. _xacro-file-reference-conditional-blocks:

Conditional Blocks
------------------

In xacro, you can use conditional branching. The xacro:if tag and xacro:unless tag are provided for conditional branching.

The xacro:if tag enables the part enclosed by the tag (inside the block) only when its value attribute is 1 or true. Conversely, the xacro:unless tag enables the part enclosed by the tag (inside the block) only when its value attribute is 0 or false. Note that both tags return an error when the value attribute takes a value other than 1/0 or true/false.

By using math macros within the value attribute, you can perform branching with complex conditions.

Example::

    <xacro:property name="var" value="useit"/>
    <xacro:if value="${var == 'useit'}"/>
    <xacro:if value="${var.startswith('use') and var.endswith('it')}"/>

    <xacro:property name="allowed" value="${[1,2,3]}"/>
    <xacro:if value="${1 in allowed}"/>

.. _xacro-file-reference-ros-commands:

ROS Commands
------------

In xacro, you can use some of the commands used in ROS (Robot Operating System) inside dollared-parentheses ($()).

The commands available in Choreonoid are as follows. All of these can be used regardless of whether you have a ROS environment or not.

env command
~~~~~~~~~~~

Using $(env ENVIRONMENT_VARIABLE) retrieves the value of the environment variable ENVIRONMENT_VARIABLE. If the environment variable does not exist, it returns an error.

optenv command
~~~~~~~~~~~~~~
Using $(optenv ENVIRONMENT_VARIABLE) retrieves the value of the environment variable ENVIRONMENT_VARIABLE. If the environment variable does not exist, it returns an empty string.

Also, if you use $(optenv ENVIRONMENT_VARIABLE default_value), it returns default_value as the default value if the environment variable does not exist.

find command
~~~~~~~~~~~~

Using $(find pkg) searches for a path ending with pkg in the environment variable ROS_PACKAGE_PATH. If no such path exists, the command returns an error.

In a ROS environment, the environment variable ROS_PACKAGE_PATH is set and you can reference installed packages. Even without a ROS environment, you can use this command by setting the environment variable ROS_PACKAGE_PATH.

arg command
~~~~~~~~~~~

Using $(arg arg1) allows you to use the argument arg1 given by the xacro:arg tag.

Example::

    <xacro:arg name="link_name" default="default_link"/>

    <link name="$(link_name)">

eval command
~~~~~~~~~~~~

Using $(eval <expression>) allows you to evaluate complex expressions that cannot be handled with regular dollared-braces (${}).

Example (combining commands and string concatenation)::

    <xacro:property name="paths" value="$(eval env('PATH') + ':' + find('pkg')">

.. _xacro-file-reference-macro:

Macros
------

The most powerful feature of xacro is macros. Macros are defined using the xacro:macro tag. Specify the macro name with the name attribute and the macro parameters (equivalent to function arguments) with the params attribute. When there are multiple parameters, list them separated by spaces.

Each parameter takes a string by default, but tags or blocks (multiple tags enclosed by a certain tag) can also be used as parameters. By adding one asterisk (*) before the parameter name, you can give XML tags as parameters, and by adding two asterisks, you can give XML blocks as parameters.

When giving tags or blocks as parameters, there is no need to associate the parameter name with the tag name or block name to be given. However, when giving multiple tags or blocks as parameters, the order of parameters corresponds to the order written, so care is needed. In the following example, the two blocks b0 and abc are expanded corresponding to block0 and block1 respectively.

Example::

    <robot name="sample" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:macro name="sample_macro" params="prefix *tag **block0 **block1">
        <link name="${prefix}_link">
          <inertial>
            <xacro:insert_block name="tag"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
          </inertial>

          <xacro:insert_block name="block0"/>

          <xacro:insert_block name="block1"/>
        </link>
      </xacro:macro>

      <xacro:sample_macro prefix="sample">
        <mass value="1.0"/>
        <b0>
          <collision>
            <geometry>
              <box size="1.0 1.0 1.0"/>
            </geometry>
          </collision>
          <!-- memo -->
        </b0>
        <abc>
          <visual>
            <geometry>
              <box size="0.5 0.5 0.5"/>
            </geometry>
            <material>
              <color rgba="1.0 0.0 0.0 1.0"/>
            </material>
          </visual>
        </abc>
      </xacro:sample_macro>
    </robot>


xacro output::

    <robot name="sample">
      <link name="sample_link">
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
        <collision>
          <geometry>
            <box size="1.0 1.0 1.0"/>
          </geometry>
        </collision>
        <!-- memo -->
        <visual>
          <geometry>
            <box size="0.5 0.5 0.5"/>
          </geometry>
          <material>
            <color rgba="1.0 0.0 0.0 1.0"/>
          </material>
        </visual>
      </link>
    </robot>

Macros within Macros
~~~~~~~~~~~~~~~~~~~~

Macros can contain other macros inside them. However, the inner macros must be defined beforehand.

Example::

    <robot name="sample" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <link name="a">
        <xacro:macro name="mass" params="value">
          <mass value="${value}"/>
        </xacro:macro>

        <xacro:macro name="inertial">
          <inertial>
            <xacro:mass value="1.0"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
          </inertial>
        </xacro:macro>

        <xacro:inertial/>
      </link>
    </robot>

xacro output::

    <robot name="sample">
      <link name="a">
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>
    </robot>

Scope
~~~~~

The scope of variables and macros is basically inside the macro. In other words, variables and macros defined inside a macro can, in principle, only be used inside that macro.

When you absolutely need to reference external variables or macros, you can extend the scope to one level up (parent) by adding the attribute scope="parent" to the tags that define those variables or macros. Alternatively, by adding the attribute scope="global", you can extend your scope to global, that is, to the entire document. However, the more you extend the scope, the more complex name management becomes, so care must be taken when using these scope extensions.

Default Values for Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Standard (string) parameters of macros can have default values. Default values are set by following the parameter name with := when defining the macro.

Also, by using ^ (circumflex) and following the parameter name with :=^, you can read an external variable with the same name. Furthermore, like :=^| 1.0, it first looks for an external variable, and if not found, the value following the vertical bar (1.0 in this case) is used as the default value.

Example::

    <robot name="sample" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <link name="a">
        <xacro:property name="y" value="10.0"/>

        <xacro:macro name="inertial" params="x:=1.0 y:=^ z:=^|3.0">
          <inertial>
            <origin xyz="${x} ${y} ${z}"/>
            <mass value="1.0"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
          </inertial>
        </xacro:macro>

        <xacro:inertial/>
      </link>
    </robot>

xacro output::

    <robot name="sample">
      <link name="a">
        <inertial>
          <origin xyz="1.0 10.0 3.0"/>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>
    </robot>

.. _xacro-file-reference-loading-files:

Loading Files
-------------

Using the xacro:include tag allows you to load other xacro files.

Example::

    <xacro:include filename="$(find package)/other_file.xacro"/>

To avoid name conflicts for macros and variables, you can also add a namespace with the ns attribute when loading.

Example (adding namespace)::

    <xacro:include filename="$(find package)/other_file.xacro" ns="namespace/>

.. _xacro-file-reference-processing-order:

Processing Order
----------------

The xacro command reads the given file from top to bottom and processes and evaluates it sequentially.

Comparison with Past Specifications
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Initially, the xacro command processed xacro files in the following order:

1. File loading (expansion of xacro:include tags)
2. Definition of properties and macros
3. Macro expansion
4. Evaluation of mathematical expressions, macros, etc.

Since evaluation was the last process, conditional branching with if or unless did not affect the definition of variables or macros. By changing the processing order from this old specification to the current specification, the following advantages were realized:

- File loading and definition of variables and macros can be executed through conditional branching. Therefore, only necessary file loading and variable/macro definitions can be executed.
- The name of the file to be loaded can be specified using variables, macros, etc.
- If you change the value of a variable in the middle of a file, the change is reflected only in the parts after that.
- Local macros and variables can be defined without affecting variables with the same name in other scopes.