# Package Border

This package is not a ROS package to launch but a collection of classes that will be called by the Openflow server.

Border.cpp is a superclass encapsulating general operations such as the loading of depthmap parameters and some transforms from depthmap to robot space and vice versa.

# StaticBorder

This class create static borders. They are zones display and monitored in fromt of the operator. They  have the purpose to contains objects that the robot will place inside so the operator can pick them up.

# StaticBorderManager

This class is a manager of StaticBorder, supporting the creation and monitoring of several static borders.

# DynamicBorder

This class creates and monitor a dynamic border around the robot (HRC). It draws a line around it and moves when the robot is in motion. If someone crosses the line, it raises a violation event to Openflow.