#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include <QObject>
#include <QProcess>

class Camera_Manager : public QObject
{
    Q_OBJECT

public:
    explicit Camera_Manager(QObject *parent = nullptr);

    void startCamera();
    void stopCamera();

signals:
    void newLog(QString text);

private:
    QProcess *process_;

};

#endif
