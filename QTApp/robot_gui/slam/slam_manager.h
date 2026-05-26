#ifndef SLAM_MANAGER_H
#define SLAM_MANAGER_H

#include <QObject>
#include <QProcess>

class SlamManager : public QObject
{
    Q_OBJECT

public:
    explicit SlamManager(QObject *parent = nullptr);

    void SlamToolBox();
    void stop();

signals:
    void newLog(QString text);

private:
    QProcess *slam_process_;

};

#endif
