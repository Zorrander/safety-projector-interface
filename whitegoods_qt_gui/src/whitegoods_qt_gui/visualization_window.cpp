#include "whitegoods_qt_gui/visualization_window.h"
#include "ui_visualization_window.h"

VisualizationWindow::VisualizationWindow(QWidget *parent, QNode *qnode) :
    QDialog(parent),
    qnode(qnode),
    ui(std::make_unique<Ui::VisualizationWindow>()) 
{
    ui->setupUi(this);

}

VisualizationWindow::~VisualizationWindow() {}
