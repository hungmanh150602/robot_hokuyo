#ifndef LIDAR_MANAGER_H
#define LIDAR_MANAGER_H

#include <QObject>
#include <QProcess>

class LidarManager : public QObject
{
    Q_OBJECT

public:
    explicit LidarManager(QObject *parent = nullptr);

    void startLidar();
    void stop();

signals:
    void newLog(QString text);

private:
    QProcess *process_;
};

#endif
