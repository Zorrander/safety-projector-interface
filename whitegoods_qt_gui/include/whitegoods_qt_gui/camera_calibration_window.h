#ifndef CAMERA_CALIBRATION_WINDOW_H
#define CAMERA_CALIBRATION_WINDOW_H

#include <QDialog>
#include "whitegoods_qt_gui/qnode.h"

namespace Ui {
class CameraCalibrationWindow;
}

class CameraCalibrationWindow : public QDialog {
    Q_OBJECT

public:
    explicit CameraCalibrationWindow(QWidget *parent = 0, QNode *qnode = nullptr);
    ~CameraCalibrationWindow();

public slots:
    void onCalibrateButtonClicked();

private:
	std::unique_ptr<Ui::CameraCalibrationWindow> ui;
    QNode *qnode;  // Reference to the ROS node
};

#endif // CAMERA_CALIBRATION_WINDOW_H