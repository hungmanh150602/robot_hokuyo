#include "person_manager.h"
#include "config_path.h"

PersonManager::PersonManager(QObject *parent)
    : QObject(parent)
{
    personTracker_process_ = new QProcess(this);

    personTracker_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    connect(personTracker_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(personTracker_process_->readAll()));
    });
}

void PersonManager::startPersonTracker()
{
    if (personTracker_process_->state() != QProcess::NotRunning)
    {
        return;
    }

    QString program = "bash";
    QStringList arguments;

    arguments << "-c"
              << src_ros + " && "
                 + src_ws + " && " + persontracker_run;

    personTracker_process_->start(program, arguments);

    if (!personTracker_process_->waitForStarted())
    {
        return;
    }
}

void PersonManager::stop()
{
    if(personTracker_process_->state() != QProcess::NotRunning)
    {
        personTracker_process_->terminate();

        if(!personTracker_process_->waitForFinished(3000))
        {
            personTracker_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "leg_follower_node");
}
