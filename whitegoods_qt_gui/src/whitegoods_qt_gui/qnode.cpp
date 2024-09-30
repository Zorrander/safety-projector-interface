#include "whitegoods_qt_gui/qnode.h"

QNode::QNode() {}

QNode::~QNode() {
    if (ros::isStarted()) {
        ros::shutdown(); // Shutdown ROS node
    }
}

bool QNode::init() {
    ros::start();
    ros::NodeHandle nh;

    return true;
}

void QNode::run() {
    ros::spin();
}

