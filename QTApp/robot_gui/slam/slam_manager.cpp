#include "slam_manager.h"
#include "config_path.h"

SlamManager::SlamManager(QWidget *parent)
    : QObject(parent), parent_widget_(parent)
{
    slam_process_ = new QProcess(this);
    save_map_process_ = new QProcess(this);
    load_map_process_ = new QProcess(this);
    configure_active_process_ = new QProcess(this);

    slam_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    save_map_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    load_map_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    configure_active_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr

    connect(slam_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(slam_process_->readAll()));
    });

    connect(save_map_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(save_map_process_->readAll()));
    });

    connect(load_map_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(load_map_process_->readAll()));
    });

    connect(configure_active_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(configure_active_process_->readAll()));
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
              << src_ros + " && "
                 + src_ws + " && "
                 "ros2 launch slam_toolbox online_async_launch.py";

    slam_process_->start(program, arguments);

    if(!slam_process_->waitForStarted())
    {
        return;
    }
}

void SlamManager::saveMap()
{
    QString file_path = QFileDialog::getSaveFileName(
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
                src_ros + " && "
                + src_ws + " && "
                "ros2 run nav2_map_server map_saver_cli"
                " -f " + file_path;

        QStringList arguments;

        arguments << "-c" << command;

        save_map_process_->start("bash", arguments);
}

void SlamManager::checkUseSimTime(bool checked)
{
    use_sim_time_ = checked;
}

void SlamManager::loadMap()
{
    QString file_path = QFileDialog::getOpenFileName(
                parent_widget_,
                "Load Map",
                QDir::homePath(),
                "YAML Files (*.yaml)");

        if(file_path.isEmpty())
        {
            return;
        }

        QString command =
                src_ros + " && "
                + src_ws + " && "
                "ros2 run nav2_map_server map_server"
                " --ros-args "
                " -p yaml_filename:=" + file_path;

        if(use_sim_time_) {
            command += " -p use_sim_time:=true";
        }
        else {
            command += " -p use_sim_time:=false";
        }

        QStringList arguments;

        arguments << "-c" << command;

        load_map_process_->start("bash", arguments);

        // wait 3s then config
        QTimer::singleShot(
            3000,
            parent_widget_,
            [=]()
        {
            QString command =
                    src_ros + " && "
                    + src_ws + " && "
                    + "ros2 lifecycle set /map_server configure";
            command += " && sleep 3 && ";
            command += "ros2 lifecycle set /map_server activate";

            QStringList arguments;

            arguments << "-c" << command;

            configure_active_process_->start("bash", arguments);
        });
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

    if(load_map_process_->state() != QProcess::NotRunning)
    {
        load_map_process_->terminate();

        if(!load_map_process_->waitForFinished(3000))
        {
            load_map_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "map_server");
}
