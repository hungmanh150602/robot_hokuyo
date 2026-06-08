#include "slam_manager.h"
#include "config_path.h"

SlamManager::SlamManager(QWidget *parent)
    : QObject(parent), parent_widget_(parent)
{
    slam_process_ = new QProcess(this);
    save_map_process_ = new QProcess(this);
    load_map_process_ = new QProcess(this);
    configure_active_process_ = new QProcess(this);
    amcl_process_ = new QProcess(this);
    amcl_configure_active_process_ = new QProcess(this);
    nav2_process_ = new QProcess(this);

    slam_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    save_map_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    load_map_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    configure_active_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    amcl_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    amcl_configure_active_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    nav2_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr

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

    connect(amcl_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(amcl_process_->readAll()));
    });

    connect(amcl_configure_active_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(amcl_configure_active_process_->readAll()));
    });

    connect(nav2_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(nav2_process_->readAll()));
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
                 "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=" + slam_fileName;

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
    QString command =
            src_ros + " && "
            + src_ws + " && "
            "ros2 run nav2_map_server map_server"
            " --ros-args "
            " -p yaml_filename:=" + map_fileName;

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
                    "ros2 lifecycle set /map_server configure"
                    " && sleep 3 && "
                    "ros2 lifecycle set /map_server activate";

            QStringList arguments;

            arguments << "-c" << command;

            configure_active_process_->start("bash", arguments);
    });
}

void SlamManager::amcl_run()
{
    QString command =
            src_ros + " && "
            + src_ws + " && "
            "ros2 run nav2_amcl amcl"
            " --ros-args ";

    if(use_sim_time_) {
        command += "-p use_sim_time:=true";
    }
    else {
        command += "-p use_sim_time:=false";
    }

    command += " -p transform_tolerance:=2.0"
               " -p base_frame_id:=base_footprint"
               " -p odom_frame_id:=odom"
               " -p map_frame_id:=map";

    QStringList arguments;

    arguments << "-c" << command;

    amcl_process_->start("bash", arguments);

    // wait 3s then config
    QTimer::singleShot(
        3000,
        parent_widget_,
        [=]()
    {
        QString command =
                src_ros + " && "
                + src_ws + " && "
                "ros2 lifecycle set /amcl configure"
                " && sleep 3 && "
                "ros2 lifecycle set /amcl activate";

        QStringList arguments;

        arguments << "-c" << command;

        amcl_configure_active_process_->start("bash", arguments);
    });
}

void SlamManager::nav2_run()
{
    QString command =
            src_ros + " && "
            + src_ws + " && "
            "ros2 launch nav2_bringup navigation_launch.py";

    if(use_sim_time_) {
        command += " use_sim_time:=true";
    }
    else {
        command += " use_sim_time:=false";
    }

    command += " params_file:=" + nav2_fileName;

    QStringList arguments;

    arguments << "-c" << command;

    nav2_process_->start("bash", arguments);
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

    if(amcl_process_->state() != QProcess::NotRunning)
    {
        amcl_process_->terminate();

        if(!amcl_process_->waitForFinished(3000))
        {
            amcl_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "amcl");

    if(nav2_process_->state() != QProcess::NotRunning)
    {
        nav2_process_->terminate();

        if(!nav2_process_->waitForFinished(3000))
        {
            nav2_process_->kill();
        }
    }
}
