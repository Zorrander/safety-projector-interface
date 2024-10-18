#include "whitegoods_qt_gui/main_window.h"

#include <yaml-cpp/yaml.h>

#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QString>
#include <QTimer>
#include <QUiLoader>
#include <QVBoxLayout>
#include <fstream>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  ros::NodeHandle nh;
  ros::start();

  QUiLoader loader;

  QFile file(
      "/home/odin3//catkin_ws/src/whitegoods_qt_gui/"
      "resource/main_window.ui");
  file.open(QFile::ReadOnly);
  QWidget* formWidget = loader.load(&file, this);
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

  cameraCalibrationWidget =
      findChild<QStackedWidget*>("cameraCalibrationWidget");
  cameraCalibrationWidget->setCurrentIndex(0);

  marker_size = findChild<QSpinBox*>("markerSizeSpinBox");
  marker_size->setMaximum(300);
  marginRightSpinBox = findChild<QSpinBox*>("marginRightSpinBox");
  marginRightSpinBox->setMaximum(300);
  marginBottomSpinBox = findChild<QSpinBox*>("marginBottomSpinBox");
  marginBottomSpinBox->setMaximum(300);
  originXSpinBox = findChild<QSpinBox*>("originXSpinBox");
  originXSpinBox->setMaximum(5000);
  originYSpinBox = findChild<QSpinBox*>("originYSpinBox");
  originYSpinBox->setMaximum(5000);
  endXSpinBox = findChild<QSpinBox*>("endXSpinBox");
  endXSpinBox->setMaximum(5000);
  endYSpinBox = findChild<QSpinBox*>("endYSpinBox");
  endYSpinBox->setMaximum(5000);

  QTimer* rosTimer = new QTimer(this);
  connect(rosTimer, &QTimer::timeout, this, []() { ros::spinOnce(); });

  rosTimer->start(10);

  process = new QProcess(this);

  outputCameraTerminal = findChild<QTextEdit*>("outputCameraTerminal");
  outputCameraTerminal->setReadOnly(true);
  connect(process, &QProcess::readyReadStandardOutput, this,
          &MainWindow::readNodeOutput);

  rgb_label = findChild<QLabel*>("RGBLabel");
  rgb_sub = nh.subscribe("/rgb/image_raw", 1, &MainWindow::rgbCallback, this);

  depth_label = findChild<QLabel*>("depthLabel");
  depth_sub = nh.subscribe("/depth_to_rgb/image_raw", 1,
                           &MainWindow::depthCallback, this);

  camera_info_sub = nh.subscribe("/rgb/camera_info", 1,
                                 &MainWindow::cameraInfoCallback, this);
}

void MainWindow::readNodeOutput() {
  // Read and append standard output from ROS node to the terminalOutput widget
  QByteArray output = process->readAllStandardOutput();
  outputCameraTerminal->append(output);
}

void MainWindow::on_newCalibrationButton_clicked() {
  // Define the command to start the ROS launch file
  QString program = "roslaunch";
  QStringList arguments;
  arguments << "tuni_whitegoods_perception"
            << "kinect_simple.launch";

  // Start the process
  process->start(program, arguments);

  if (process->waitForStarted()) {
    QMessageBox::information(this, "ROS Launch",
                             "ROS node launched successfully.");
  } else {
    QMessageBox::warning(this, "ROS Launch", "Failed to launch the ROS node.");
  }

  mainWidget->setCurrentIndex(1);
}

void MainWindow::on_editCalibrationButton_clicked() {
  mainWidget->setCurrentIndex(2);
}

void MainWindow::on_calibrateRobotButton_clicked() {
  stackedWidget->setCurrentIndex(0);
}

void MainWindow::on_calibrateCameraButton_clicked() {
  stackedWidget->setCurrentIndex(1);
}

void MainWindow::on_calibrateProjectorButton_clicked() {
  stackedWidget->setCurrentIndex(2);
}

void MainWindow::on_cameraParametersButton_clicked() {
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

void MainWindow::on_updateCameraCalibrationButton_clicked() {
  on_cameraParametersButton_clicked();
}

void MainWindow::on_confirmCameraCalibrationButton_clicked() {
  YAML::Node calib;
  calib["camera_resolution"].push_back(camera_width);
  calib["camera_resolution"].push_back(camera_height);
  calib["fx"] = fx;
  calib["fy"] = fy;
  calib["cx"] = cx;
  calib["cy"] = cy;

  QString defaultPath = QDir::homePath();

  QString fileName = QFileDialog::getSaveFileName(
      this, "Save RGB image", defaultPath + "/.yaml",
      "YAML Files (*.yaml);;All Files (*)");

  if (!fileName.isEmpty()) {
    std::ofstream fout(fileName.toStdString());
    fout << calib;
  }
}

void MainWindow::on_projectMarkersButton_clicked() {
  cv::Ptr<cv::aruco::Dictionary> aruco_dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  cv::Mat testim = cv::Mat::zeros(1080, 1920, CV_8UC1) + 255;

  cv::namedWindow("window", cv::WINDOW_NORMAL);
  cv::setWindowProperty("window", cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  cv::moveWindow("window", 2560, 0);

  int size = marker_size->value();
  int aruc_id = 0;

  std::map<int, cv::Point2f> predefinedPtsSrc;

  // Generate ArUco markers and place them in the image
  for (int y = originYSpinBox->value(); y < endYSpinBox->value();
       y += marginBottomSpinBox->value()) {
    for (int x = originXSpinBox->value(); x < endXSpinBox->value();
         x += marginRightSpinBox->value()) {
      // Create a blank marker image (size x size)
      cv::Mat tag(size, size, CV_8UC1, cv::Scalar(255));

      // Generate an ArUco marker with the current ID
      cv::aruco::drawMarker(aruco_dict, aruc_id, size, tag);

      // Place the marker in the test image
      tag.copyTo(testim(cv::Rect(x, y, size, size)));

      // Draw the corner coordinates text just above the marker
      // cv::circle(testim, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), 2); //
      // Red circles

      predefinedPtsSrc[aruc_id] = cv::Point2f(x, y);

      // Increment the ArUco marker ID
      aruc_id++;
    }
  }

  // Display the final image with markers
  cv::imshow("window", testim);
  cv::waitKey(1500);

  std::vector<std::vector<cv::Point2f>> markerCorners;
  std::vector<int> markerIds;

  cv::Mat copiedImage = cv_rgb.clone();

  // Detect ArUco markers
  cv::aruco::detectMarkers(copiedImage, aruco_dict, markerCorners, markerIds);

  std::vector<cv::Point2f> pts_src;
  std::vector<cv::Point2f> pts_dst;

  // Draw detected markers and their axes
  if (!markerIds.empty()) {
    for (size_t i = 0; i < markerIds.size(); i++) {
      cv::circle(copiedImage, markerCorners[i][0], 10, cv::Scalar(0, 0, 255),
                 2);  // Red circles
      pts_dst.push_back(predefinedPtsSrc[markerIds[i]]);
      pts_src.push_back(
          cv::Point2f(markerCorners[i][0].x, markerCorners[i][0].y));
    }
  }

  cv::namedWindow("ROS Image", cv::WINDOW_NORMAL);
  cv::imshow("ROS Image", copiedImage);
  cv::waitKey(0);

  cv::Mat homography = cv::findHomography(pts_src, pts_dst);

  QString defaultPath = QDir::homePath();

  QString fileName = QFileDialog::getSaveFileName(
      this, "Save homography matrix", defaultPath + "/.yaml",
      "YAML (*.yaml);;All Files (*)");

  cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::WRITE);

  if (fs.isOpened()) {
    // Write the homography matrix to the file
    fs << "homography" << homography;
    fs.release();  // Release the file
    std::cout << "Homography saved to " << fileName.toStdString() << std::endl;
  } else {
    std::cerr << "Error: Unable to open file for writing: "
              << fileName.toStdString() << std::endl;
  }

  cv::Mat warped;
  cv::warpPerspective(copiedImage, warped, homography, copiedImage.size());

  // Display the warped result for visualization (optional)
  cv::namedWindow("Warped Image", cv::WINDOW_NORMAL);
  cv::imshow("Warped Image", warped);
  cv::waitKey(0);
}

void MainWindow::on_captureRGBButton_clicked() {
  QString defaultPath = QDir::homePath();

  QString fileName = QFileDialog::getSaveFileName(
      this, "Save RGB image", defaultPath + "/.png",
      "Text Files (*.png);;All Files (*)");

  if (!fileName.isEmpty()) {
    cv::imwrite(fileName.toStdString(), cv_rgb);
  }
  QPixmap pixmap(fileName);
  rgb_label->setPixmap(pixmap.scaled(rgb_label->size(), Qt::KeepAspectRatio,
                                     Qt::SmoothTransformation));
  rgbWidget->setCurrentIndex(1);
}

void MainWindow::on_captureDepthButton_clicked() {
  QString defaultPath = QDir::homePath();

  QString fileName = QFileDialog::getSaveFileName(
      this, "Save Depth image", defaultPath + "/.png",
      "Text Files (*.png);;All Files (*)");

  cv::normalize(cv_depth, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);

  if (!fileName.isEmpty()) {
    cv::imwrite(fileName.toStdString(), depth_colormap);
  }

  QPixmap pixmap(fileName);
  depth_label->setPixmap(pixmap.scaled(depth_label->size(), Qt::KeepAspectRatio,
                                       Qt::SmoothTransformation));
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

void MainWindow::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
  camera_width = msg->width;
  camera_height = msg->height;
  fx = msg->K[0];
  fy = msg->K[4];
  cx = msg->K[2];
  cy = msg->K[5];
}
