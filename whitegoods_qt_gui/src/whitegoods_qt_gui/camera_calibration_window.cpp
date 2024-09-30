#include "whitegoods_qt_gui/camera_calibration_window.h"
#include "ui_camera_calibration_window.h"

CameraCalibrationWindow::CameraCalibrationWindow(QWidget *parent, QNode *qnode) :
    QDialog(parent),
    qnode(qnode),
    ui(std::make_unique<Ui::CameraCalibrationWindow>()) 
{
    ui->setupUi(this);

    // Connect the calibration button to the callback
    connect(ui->calibrateButton, &QPushButton::clicked, this, &CameraCalibrationWindow::onCalibrateButtonClicked);
}

CameraCalibrationWindow::~CameraCalibrationWindow() {}

void CameraCalibrationWindow::onCalibrateButtonClicked() {
    // Example: Publish a message or call a service for calibration
    std_msgs::Bool msg;
    msg.data = true;
    qnode->getCalibrationPublisher().publish(msg);
}