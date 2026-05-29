#ifndef ROBOT_MANAGEHR_H
#define ROBOT_MANAGEHR_H

#include <QObject>
#include <QProcess>

class RobotManager : public QObject
{
    Q_OBJECT

public:
    explicit RobotManager(QObject *parent);
    void loadRobotModel();
    void stop();

signals:
    void newLog(QString text);

private:
    QProcess *robot_process_;
    QProcess *map_tf_process_;

};

#endif
