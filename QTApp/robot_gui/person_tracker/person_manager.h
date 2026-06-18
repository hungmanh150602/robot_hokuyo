#ifndef PERSON_MANAGEHR_H
#define PERSON_MANAGEHR_H

#include <QObject>
#include <QProcess>

class PersonManager : public QObject
{
    Q_OBJECT

public:
    explicit PersonManager(QObject *parent);

    void startPersonTracker();
    void stop();

signals:
    void newLog(QString text);

private:
    QProcess *personTracker_process_;

};

#endif
