==============================
Custom Toolbar Sample (S04)
==============================

.. contents:: Table of Contents
   :local:

Overview
--------

As a practical example of toolbar creation introduced in the previous section, we present a sample plugin that uses a toolbar. This is an extension of the previous sample :doc:`item-operation-sample`, with the same functionality of rotating selected Body item models, but creating a toolbar related to this function and enabling additional operations and settings using toolbar buttons and other elements.

Source Code
-----------

.. highlight:: cpp

Here is the source code for this sample. As before, create a plugin source directory and store the following source code with the filename DevGuidePlugin.cpp: ::

 #include <cnoid/Plugin>
 #include <cnoid/ItemList>
 #include <cnoid/RootItem>
 #include <cnoid/BodyItem>
 #include <cnoid/ToolBar>
 #include <cnoid/TimeBar>
 #include <cnoid/DoubleSpinBox>
 #include <cnoid/EigenTypes>
 #include <vector>
 #include <cmath>
 
 using namespace cnoid;
 
 class DevGuidePlugin : public Plugin
 {
     ItemList<BodyItem> bodyItems;
     double initialTime;
     std::vector<Matrix3> initialRotations;
     DoubleSpinBox* speedRatioSpin;
     ToolButton* reverseToggle;
 
 public:
     DevGuidePlugin() : Plugin("DevGuide")
     {
         require("Body");
     }
 
     virtual bool initialize() override
     {
         RootItem::instance()->sigSelectedItemsChanged().connect(
             [this](const ItemList<>& selectedItems){
                 onSelectedItemsChanged(selectedItems);
             });
                    
         TimeBar::instance()->sigTimeChanged().connect(
             [this](double time){
                 return onTimeChanged(time);
             });
 
         auto toolBar = new ToolBar("DevGuideBar");

         ToolButton* flipButton = toolBar->addButton("Flip");
         flipButton->sigClicked().connect(
             [this](){ flipBodyItems(); });
 
         reverseToggle = toolBar->addToggleButton("Reverse");
         reverseToggle->sigToggled().connect(
             [this](bool on){ updateInitialRotations(); });
 
         toolBar->addSeparator();
 
         toolBar->addLabel("Speed ratio");
         speedRatioSpin = new DoubleSpinBox;
         speedRatioSpin->setValue(1.0);
         speedRatioSpin->sigValueChanged().connect(
             [this](double value){ updateInitialRotations(); });
         toolBar->addWidget(speedRatioSpin);
 
         toolBar->setVisibleByDefault();
         addToolBar(toolBar);
         
         initialTime = 0.0;
 
         return true;
     }
 
     void onSelectedItemsChanged(ItemList<BodyItem> selectedBodyItems)
     {
         if(selectedBodyItems != bodyItems){
             bodyItems = selectedBodyItems;
             updateInitialRotations();
         }
     }
 
     void updateInitialRotations()
     {
         initialTime = TimeBar::instance()->time();
         initialRotations.clear();
         for(auto& bodyItem : bodyItems){
             initialRotations.push_back(bodyItem->body()->rootLink()->rotation());
         }
     }
 
     void flipBodyItems()
     {
         for(auto& bodyItem : bodyItems){
             Link* rootLink = bodyItem->body()->rootLink();
             Matrix3 R = AngleAxis(M_PI, Vector3::UnitZ()) * rootLink->rotation();
             rootLink->setRotation(R);
             bodyItem->notifyKinematicStateChange(true);
         }
         updateInitialRotations();
     }
 
     bool onTimeChanged(double time)
     {
         for(size_t i=0; i < bodyItems.size(); ++i){
             auto bodyItem = bodyItems[i];
             double angle = speedRatioSpin->value() * (time - initialTime);
             if(reverseToggle->isChecked()){
                 angle = -angle;
             }
             Matrix3 R = AngleAxis(angle, Vector3::UnitZ()) * initialRotations[i];
             bodyItem->body()->rootLink()->setRotation(R);
             bodyItem->notifyKinematicStateChange(true);
         }
 
         return !bodyItems.empty();
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)
           
The CMakeLists.txt for building is the same as :doc:`item-operation-sample`.

Toolbar Functions
-----------------

When this plugin is loaded, the following toolbar is displayed in the toolbar area:

.. image:: images/toolbar1.png

If this is not displayed, find the "DevGuideBar" item in the main menu "View" - "Show Toolbars" and check it.

The basic function of the plugin is the same as :doc:`item-operation-sample`, rotating the model of the Body item selected in the item tree view in conjunction with the time bar time.

First, there is a "Flip" button on the toolbar. This is a normal push-type button that can be clicked with the mouse. When you press this button, the selected model rotates 180 degrees. As the name "Flip" suggests, the orientation is reversed.

Next, there is a "Reverse" button. This is a button that reverses the rotation direction relative to time progression. This is a toggle button, and when you click with the mouse, it becomes pressed and the button is in the ON state. In this state, the rotation direction is reversed. This is easier to understand during animation, so try executing animation with the time bar play button and pressing it during the animation.

At the end of the toolbar, there is "Speed ratio" label with a numeric input box. This is a box for setting the rotation ratio relative to time progression. If you decrease the value here, the rotation becomes slower, and if you increase it, the rotation becomes faster. This is also easier to understand if you change it during animation. When this is 1.0, it's the same as :doc:`item-operation-sample`, where the time in seconds is directly treated as the rotation angle in radians, but the Speed ratio setting allows you to change the ratio of angle to seconds.

As usual, this itself has no particular meaning, but it serves as a sample that uses the basic functions of toolbars comprehensively. The two buttons are examples of using the button and toggle button introduced in :ref:`plugin-dev-toolbar-functions`, and it also uses separators and labels, with the last numeric input box being an example of :ref:`plugin-dev-toolbar-use-qt-classes`.

Source Code Explanation
-----------------------

We'll explain mainly the parts added to :doc:`item-operation-sample`. ::

 #include <cnoid/ToolBar>

This is the header for the ToolBar class. Include this header when creating toolbars. In this sample, we're also using TimeBar, and its header also includes the ToolBar header, but here we're explicitly including the ToolBar header as part of the toolbar creation procedure. ::

 #include <cnoid/DoubleSpinBox>

We're using DoubleSpinBox to implement the numeric box for entering Speed ratio. This extends Qt's QDoubleSpinBox to enable use of Choreonoid-format signals. QDoubleSpinBox is a spin box widget that can handle double-precision floating-point numbers. ::

 #include <cmath>

This is the C language math header. We're including this to use the M_PI macro, which has the value of pi.

The following are additions to the plugin's member variables: ::

 double initialTime;

A variable to store the initial time for determining the rotation amount relative to time.
This is introduced to maintain smooth continuous animation even when rotation settings are changed.

 DoubleSpinBox* speedRatioSpin;

A pointer to the Speed ratio spin box. This is defined to reference the created spin box from other member functions. ::
 
 ToolButton* reverseToggle;

A pointer to the toggle button. This is also defined to get the button state from other member variables after creation.

Next, we enter the implementation of the initialization function initialize: ::

 RootItem::instance()->sigSelectedItemsChanged().connect(
     [this](const ItemList<>& selectedItems){
         onSelectedItemsChanged(selectedItems);
     });
 
 TimeBar::instance()->sigTimeChanged().connect(
     [this](double time){
         return onTimeChanged(time);
     });

This part is basically the same as :doc:`item-operation-sample`, but in this sample we've omitted signal connection management with ScopedConnectionSet.
We included it as an example of connection management in the previous sample, but in this case, the connections are maintained until the application ends, so connection management is not necessarily required.
We'll also omit unnecessary parts in future sample code. ::

 auto toolBar = new ToolBar("DevGuideBar");

Creating the toolbar object. The ToolBar constructor requires a name, which we set as "DevGuideBar" here. In this sample, we're coding using the "create an instance of the ToolBar class and add necessary interfaces from outside" method among the two creation methods introduced in :doc:`toolbar`. ::

 ToolButton* flipButton = toolBar->addButton("Flip");
 flipButton->sigClicked().connect(
     [this](){ flipBodyItems(); });

Adding a Flip button to the toolbar and connecting the member function flipBodyItems to its sigClicked signal.
This causes flipBodyItems to be executed when the Flip button is pressed. ::

 reverseToggle = toolBar->addToggleButton("Reverse");
 reverseToggle->sigToggled().connect(
     [this](bool on){ updateInitialRotations(); });

Adding a Reverse toggle button to the toolbar and connecting the member function updateInitialRotations to its sigToggled signal.
This causes updateInitialRotations to be executed when the Reverse toggle's on/off state changes, resetting the current state as the initial state. ::
     
 toolBar->addSeparator();
  
Adding a separator to the toolbar.
Since the toggle button's label and the label to be added next are both text and would appear connected, we're adding a separator to make the boundary clearer.
This is purely a matter of visual preference. ::

 toolBar->addLabel("Speed ratio");

Adding the label "Speed ratio" to the toolbar. If we only add the spin box next, it's unclear what the box is for, so we're adding a label. ::

 speedRatioSpin = new DoubleSpinBox;

Creating a DoubleSpinBox as a spin box for numeric input. ::
   
 speedRatioSpin->setValue(1.0);
 
Setting the initial value of the spin box to 1.0. This is a function of QDoubleSpinBox, the Qt class that DoubleSpinBox inherits from. ::

 speedRatioSpin->sigValueChanged().connect(
      [this](double value){ updateInitialRotations(); });

Connecting the sigValueChanged signal that is emitted when the spin box value changes.
sigValueChanged makes the QDoubleSpinBox's valueChanged signal available as a Choreonoid signal.
This also causes updateInitialRotations to be executed when the value changes, updating the initial state. ::

 toolBar->addWidget(speedRatioSpin);

Adding the created spin box to the toolbar. ::

 toolBar->setVisibleByDefault();
  
Making this toolbar visible by default.
If not specified, it's set to not be displayed by default.
Toolbar layout is often determined by project settings, so normally it's set to not display.
Since this is a sample, it's preferable that the created toolbar is displayed from the beginning for easy operation verification, so we're deliberately making this setting. ::

 addToolBar(toolBar);
 
Registering the created toolbar. This process makes the toolbar available for use. ::

 initialTime = 0.0;

Initializing the initial time for rotation amount determination to 0.

The following is a function to update the initial state for rotation amount determination: ::

 void updateInitialRotations()
 {
     initialTime = TimeBar::instance()->time();
     initialRotations.clear();
     for(auto& bodyItem : bodyItems){
         initialRotations.push_back(bodyItem->body()->rootLink()->rotation());
     }
 }

First, update the value of initialTime with the current time. The rotation amount is calculated using time starting from this time.
The remaining part is what was processed in onSelectedItemsChanged in :doc:`item-operation-sample`.
By consolidating it in this function, we can update the initial state even when settings are changed.

The following is the function called when the Flip button is pressed: ::

 void flipBodyItems()
 {
     for(auto& bodyItem : bodyItems){
         Link* rootLink = bodyItem->body()->rootLink();
         Matrix3 R = AngleAxis(M_PI, Vector3::UnitZ()) * rootLink->rotation();
         rootLink->setRotation(R);
         bodyItem->notifyKinematicStateChange(true);
     }
     updateInitialRotations();
 }

For selected Body items, rotate the root link's orientation by 180 degrees.
Finally, also execute updateInitialRotations to ensure that rotation due to time changes continues after this process.

The onTimeChanged function determines the rotation amount according to time changes and updates the model state.
This basic process is the same as :doc:`item-operation-sample`, but we've modified the formula for determining the rotation amount.

First: ::

 double angle = speedRatioSpin->value() * (time - initialTime);

Here, we make the time relative to initialTime, then multiply it by the Speed ratio spin box value to determine the rotation angle.
This achieves smooth changes from the time of setting changes while also reflecting the Speed ratio. ::

 if(reverseToggle->isChecked()){
     angle = -angle;
 }

Check the state of the Reverse toggle button, and if it's on, reverse the rotation angle to the negative direction.
This makes the Reverse toggle button functional.