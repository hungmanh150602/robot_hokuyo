#include "slam_manager.h"

SlamManager::SlamManager(QWidget *parent)
    : QObject(parent), parent_widget_(parent)
{
    slam_process_ = new QProcess(this);
    save_map_process_ = new QProcess(this);

    slam_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    save_map_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr

    connect(slam_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(slam_process_->readAll()));
    });

    connect(save_map_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(save_map_process_->readAll()));
    });
}

void SlamManager::SlamToolBox()
{
    /* Already running */
    if(slam_process_->state() != QProcess::NotRunning)
    {
        return;
    }

    QString program = "bash";
    QStringList arguments;

    arguments << "-c"
              << "source /opt/ros/humble/setup.bash && "
                 "source ~/ros2_workspace/install/setup.bash && "
                 "ros2 launch slam_toolbox online_async_launch.py";

    slam_process_->start(program, arguments);

    if(!slam_process_->waitForStarted())
    {
        return;
    }
}

void SlamManager::stop()
{
    if(slam_process_->state() != QProcess::NotRunning)
    {
        slam_process_->terminate();

        if(!slam_process_->waitForFinished(3000))
        {
            slam_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "slam_toolbox");
}

void SlamManager::saveMap()
{
    QString file_path =
            QFileDialog::getSaveFileName(
                parent_widget_,
                "Save Map",
                QDir::homePath(),
                "YAML Files (*.yaml)");

        if(file_path.isEmpty())
        {
            return;
        }

        /* Remove .yaml */
        if(file_path.endsWith(".yaml"))
        {
            file_path.chop(5);
        }

        QString command =
                "source /opt/ros/humble/setup.bash && "
                "source ~/ros2_workspace/install/setup.bash && "
                "ros2 run nav2_map_server map_saver_cli "
                "-f " + file_path;

        QStringList arguments;

        arguments << "-c" << command;

        save_map_process_->start("bash", arguments);
}
