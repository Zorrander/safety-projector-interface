#ifndef VISUALIZATION_WINDOW_H
#define VISUALIZATION_WINDOW_H

#include <QDialog>
#include "whitegoods_qt_gui/qnode.h"


namespace Ui {
class VisualizationWindow;
}

class VisualizationWindow : public QDialog {
    Q_OBJECT

public:
    explicit VisualizationWindow(QWidget *parent = 0, QNode *qnode = nullptr);
    ~VisualizationWindow();

private:
	std::unique_ptr<Ui::VisualizationWindow> ui;
    QNode *qnode;

public slots:
    void onNewImageReceived(const QImage &image);
    
};

#endif // VISUALIZATION_WINDOW_H