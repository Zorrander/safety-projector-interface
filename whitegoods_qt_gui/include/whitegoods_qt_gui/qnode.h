#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include <QThread>


class QNode : public QThread {
    Q_OBJECT

public:
    QNode();
    ~QNode();
    bool init();
    void run() override;

};

#endif // QNODE_H
