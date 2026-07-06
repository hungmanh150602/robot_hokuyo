#include "person_manager.h"
#include "config_path.h"

PersonManager::PersonManager(QObject *parent)
    : QObject(parent)
{
    personTracker_process_ = new QProcess(this);
    cameraDetection_process_ = new QProcess(this);

    personTracker_process_->setProcessChannelMode(QProcess::MergedChannels);   // read all: stdout stderr
    cameraDetection_process_->setProcessChannelMode(QProcess::MergedChannels); // read all: stdout stderr
    connect(personTracker_process_, &QProcess::readyRead, this, [=]()
            { emit newLog(QString::fromLocal8Bit(personTracker_process_->readAll())); });
    connect(cameraDetection_process_, &QProcess::readyRead, this, [=]()
            { emit newLog(QString::fromLocal8Bit(cameraDetection_process_->readAll())); });
}

void PersonManager::startCameraDetection()
{
    if (cameraDetection_process_->state() != QProcess::NotRunning)
    {
        return;
    }

    QString command = src_ros + " && " + src_ws + " && " + camera_detect_run;
    QStringList arguments;

    arguments << "-c" << command;
    cameraDetection_process_->start("bash", arguments);

    if (!cameraDetection_process_->waitForStarted())
    {
        return;
    }
}

void PersonManager::startPersonTracker()
{
    if (personTracker_process_->state() != QProcess::NotRunning)
    {
        return;
    }

    QString command =
        src_ros + " && " + src_ws + " && " + persontracker_run;

    QStringList arguments;

    arguments << "-c" << command;

    personTracker_process_->start("bash", arguments);

    if (!personTracker_process_->waitForStarted())
    {
        return;
    }
}

void PersonManager::stop()
{
    if (personTracker_process_->state() != QProcess::NotRunning)
    {
        personTracker_process_->terminate();

        if (!personTracker_process_->waitForFinished(3000))
        {
            personTracker_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "person_detector");

    if (cameraDetection_process_->state() != QProcess::NotRunning)
    {
        cameraDetection_process_->terminate();

        if (!cameraDetection_process_->waitForFinished(3000))
        {
            cameraDetection_process_->kill();
        }
    }
}
