#ifndef TCP_MANAGER_H
#define TCP_MANAGER_H

#include <QObject>
#include <QProcess>

class TCPManager : public QObject
{
    Q_OBJECT

public:
    explicit TCPManager(QObject *parent = nullptr);

    void startTCP();
    void stopTCP();

signals:
    void newLog(QString text);

private:
    QProcess *process_;
};

#endif
