#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <QWidget>
#include <QtUiTools>

QT_BEGIN_NAMESPACE
class QLabel;
class QStackedWidget;
class QSpinBox;
class QDoubleSpinBox;
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);

 private slots:
  void on_calibrateRobotButton_clicked();
  void on_calibrateCameraButton_clicked();
  void on_calibrateProjectorButton_clicked();
  void on_newCalibrationButton_clicked();
  void on_editCalibrationButton_clicked();
  void on_captureRGBButton_clicked();
  void on_captureDepthButton_clicked();
  void on_cameraParametersButton_clicked();
  void on_confirmCameraCalibrationButton_clicked();
  void readNodeOutput();
  void on_projectMarkersButton_clicked();
  void on_updateCameraCalibrationButton_clicked();

 private:
  QWidget *centralWidget;
  QStackedWidget *stackedWidget, *mainWidget, *rgbWidget, *depthWidget,
      *cameraCalibrationWidget;

  QLabel *rgb_label, *depth_label;
  ros::Subscriber rgb_sub, depth_sub, camera_info_sub;
  cv_bridge::CvImagePtr cv_ptr;

  cv::Mat cv_rgb, cv_depth;
  cv::Mat depth_normalized, depth_colormap;

  QTextEdit *outputCameraTerminal;
  QProcess *process;

  void rgbCallback(const sensor_msgs::ImageConstPtr &msg);
  void depthCallback(const sensor_msgs::ImageConstPtr &msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

  int camera_width, camera_height;
  double fx, fy, cx, cy;

  QSpinBox *q_cam_width, *q_cam_height, *marker_size, *marginRightSpinBox,
      *marginBottomSpinBox, *originXSpinBox, *originYSpinBox, *endXSpinBox,
      *endYSpinBox;
  QDoubleSpinBox *q_fx, *q_fy, *q_cx, *q_cy;
};

#endif  // MAIN_WINDOW_H
