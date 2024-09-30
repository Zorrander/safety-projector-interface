#include "whitegoods_qt_gui/main_window.h"

#include <QLabel>
#include <QStackedWidget>

#include <QFile>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QUiLoader>
#include <QPushButton>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ros::NodeHandle nh;
    ros::start();

    QUiLoader loader;

    QFile file("/home/odin3//catkin_ws/src/safety-projector-interface/whitegoods_qt_gui/resource/main_window.ui");
    file.open(QFile::ReadOnly);
    QWidget *formWidget = loader.load(&file, this);
    file.close();

    if (!formWidget) {
        qWarning("Failed to load the UI.");
        return;
    }

    this->setWindowTitle("Projector Interface Setup Assistant");
    centralWidget = findChild<QWidget*>("centralwidget");
    centralWidget->setMinimumSize(1280, 760);
    this->setCentralWidget(centralWidget);

    QMetaObject::connectSlotsByName(this);

    mainWidget = findChild<QStackedWidget*>("mainPages");
    mainWidget->setCurrentIndex(0);

    stackedWidget = findChild<QStackedWidget*>("hmiPages");
    stackedWidget->setCurrentIndex(0);

    rgbWidget = findChild<QStackedWidget*>("RGBWidget");
    rgbWidget->setCurrentIndex(0);

    depthWidget = findChild<QStackedWidget*>("DepthWidget");
    depthWidget->setCurrentIndex(0);

    cameraCalibrationWidget = findChild<QStackedWidget*>("cameraCalibrationWidget");
    cameraCalibrationWidget->setCurrentIndex(0);

    QTimer *rosTimer = new QTimer(this);
    connect(rosTimer, &QTimer::timeout, this, [](){
        ros::spinOnce();
    });

    rosTimer->start(10);

    rgb_label = findChild<QLabel*>("RGBLabel");
    rgb_sub = nh.subscribe("/rgb/image_raw", 1, &MainWindow::rgbCallback, this);

    depth_label = findChild<QLabel*>("depthLabel");
    depth_sub = nh.subscribe("/depth_to_rgb/image_raw", 1, &MainWindow::depthCallback, this);

    camera_info_sub = nh.subscribe("/rgb/camera_info", 1, &MainWindow::cameraInfoCallback, this);
}

void MainWindow::on_newCalibrationButton_clicked(){mainWidget->setCurrentIndex(1);}

void MainWindow::on_editCalibrationButton_clicked(){mainWidget->setCurrentIndex(2);}

void MainWindow::on_calibrateRobotButton_clicked(){stackedWidget->setCurrentIndex(0);}

void MainWindow::on_calibrateCameraButton_clicked(){stackedWidget->setCurrentIndex(1);}

void MainWindow::on_calibrateProjectorButton_clicked(){stackedWidget->setCurrentIndex(2);}

void MainWindow::on_cameraParametersButton_clicked(){
    q_cam_height = findChild<QSpinBox*>("heightSpinBox");
    q_cam_height->setMaximum(5000);
    q_cam_height->setValue(camera_height);
    q_cam_width = findChild<QSpinBox*>("widthSpinBox");
    q_cam_width->setMaximum(5000);
    q_cam_width->setValue(camera_width);
    q_fx = findChild<QDoubleSpinBox*>("fxDoubleSpinBox");
    q_fx->setMaximum(2000.0);
    q_fx->setValue(fx);
    q_fy = findChild<QDoubleSpinBox*>("fyDoubleSpinBox");
    q_fy->setMaximum(2000.0);
    q_fy->setValue(fy);
    q_cx = findChild<QDoubleSpinBox*>("cxDoubleSpinBox");
    q_cx->setMaximum(2000.0);
    q_cx->setValue(cx);
    q_cy = findChild<QDoubleSpinBox*>("cyDoubleSpinBox");
    q_cy->setMaximum(2000.0);
    q_cy->setValue(cy);
    cameraCalibrationWidget->setCurrentIndex(1);
}

void MainWindow::on_confirmCameraCalibrationButton_clicked(){
    YAML::Node calib;
    calib["value"] = 2;

    QString defaultPath = QDir::homePath();

    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save RGB image",
        defaultPath + "/.yaml",
        "YAML Files (*.yaml);;All Files (*)"
    );

    if (!fileName.isEmpty()) {
        std::ofstream fout(fileName.toStdString());
        fout << calib;
    }
}

void MainWindow::on_captureRGBButton_clicked(){
    QString defaultPath = QDir::homePath();

    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save RGB image",
        defaultPath + "/.png",
        "Text Files (*.png);;All Files (*)"
    );

    if (!fileName.isEmpty()) {
        cv::imwrite(fileName.toStdString(), cv_rgb);
    }
    QPixmap pixmap(fileName);
    rgb_label->setPixmap(pixmap.scaled(rgb_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    rgbWidget->setCurrentIndex(1);
}

void MainWindow::on_captureDepthButton_clicked(){
    QString defaultPath = QDir::homePath();

    QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save Depth image",
        defaultPath + "/.png",
        "Text Files (*.png);;All Files (*)"
    );

    cv::normalize(cv_depth, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);

    if (!fileName.isEmpty()) {
        cv::imwrite(fileName.toStdString(), depth_colormap);
    }

    QPixmap pixmap(fileName);
    depth_label->setPixmap(pixmap.scaled(depth_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    depthWidget->setCurrentIndex(1);
}



void MainWindow::rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_rgb = cv_ptr->image;
}

void MainWindow::depthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_depth = cv_ptr->image;
}

void MainWindow::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    camera_width = msg->width;
    camera_height = msg->height;
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
}

