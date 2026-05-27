#include "lidar_manager.h"

LidarManager::LidarManager(QObject *parent)
    : QObject(parent)
{
    process_ = new QProcess(this);

    process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    connect(process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(process_->readAll()));
    });
}

void LidarManager::startLidar()
{
    if (process_->state() != QProcess::NotRunning)
    {
        return;
    }

    QString program = "bash";
    QStringList arguments;

    arguments << "-c"
              << "source ~/ros2_workspace/install/setup.bash && "
                 "ros2 run robot_hokuyo publish_lidar";

    process_->start(program, arguments);

    if (!process_->waitForStarted())
    {

    }
}

void LidarManager::stop()
{
    if(process_->state() != QProcess::NotRunning)
    {
        process_->terminate();

        if(!process_->waitForFinished(3000))
        {
            process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "publish_lidar");
}
