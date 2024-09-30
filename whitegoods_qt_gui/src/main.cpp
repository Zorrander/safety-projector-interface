#include <ros/ros.h>
#include <QApplication>
#include "whitegoods_qt_gui/main_window.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_qt_app");

    QApplication app(argc, argv);

    MainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}
