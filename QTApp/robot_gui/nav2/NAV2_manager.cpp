#include "NAV2_manager.h"
#include "config_path.h"

NAV2Manager::NAV2Manager(QWidget *parent)
    : QObject(parent), parent_widget_(parent)
{
    load_map_process_ = new QProcess(this);
    configure_active_process_ = new QProcess(this);
    amcl_process_ = new QProcess(this);
    amcl_configure_active_process_ = new QProcess(this);
    nav2_process_ = new QProcess(this);

#if 1
    load_map_process_->setProcessChannelMode(QProcess::MergedChannels);              // read all: stdout stderr
    configure_active_process_->setProcessChannelMode(QProcess::MergedChannels);      // read all: stdout stderr
    amcl_process_->setProcessChannelMode(QProcess::MergedChannels);                  // read all: stdout stderr
    amcl_configure_active_process_->setProcessChannelMode(QProcess::MergedChannels); // read all: stdout stderr
    nav2_process_->setProcessChannelMode(QProcess::MergedChannels);                  // read all: stdout stderr

    connect(load_map_process_, &QProcess::readyRead, this, [=]()
            { emit newLog(QString::fromLocal8Bit(load_map_process_->readAll())); });

    connect(configure_active_process_, &QProcess::readyRead, this, [=]()
            { emit newLog(QString::fromLocal8Bit(configure_active_process_->readAll())); });

    connect(amcl_process_, &QProcess::readyRead, this, [=]()
            { emit newLog(QString::fromLocal8Bit(amcl_process_->readAll())); });

    connect(amcl_configure_active_process_, &QProcess::readyRead, this, [=]()
            { emit newLog(QString::fromLocal8Bit(amcl_configure_active_process_->readAll())); });

    connect(nav2_process_, &QProcess::readyRead, this, [=]()
            { emit newLog(QString::fromLocal8Bit(nav2_process_->readAll())); });
#endif
}

void NAV2Manager::checkUseSimTime(bool checked)
{
    use_sim_time_ = checked;
}

void NAV2Manager::loadMap()
{
    QString command = src_ros + " && " + src_ws + " && " + loadMap_run;

    if (use_sim_time_)
    {
        command += " -p use_sim_time:=true";
    }
    else
    {
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
                src_ros + " && " + src_ws + " && "
                                            "ros2 lifecycle set /map_server configure"
                                            " && sleep 1 && "
                                            "ros2 lifecycle set /map_server activate";

            QStringList arguments;

            arguments << "-c" << command;

            configure_active_process_->start("bash", arguments);
        });
}

void NAV2Manager::amcl_run()
{
    QString command = src_ros + " && " + src_ws + " && " + amcl_runn;

    if (use_sim_time_)
    {
        command += " -p use_sim_time:=true";
    }
    else
    {
        command += " -p use_sim_time:=false";
    }

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
                src_ros + " && " + src_ws + " && "
                                            "ros2 lifecycle set /amcl configure"
                                            " && sleep 1 && "
                                            "ros2 lifecycle set /amcl activate";

            QStringList arguments;

            arguments << "-c" << command;

            amcl_configure_active_process_->start("bash", arguments);
        });
}

void NAV2Manager::nav2_run()
{
    QString command = src_ros + " && " + src_ws + " && " + nav2_runn;

    if (use_sim_time_)
    {
        command += " use_sim_time:=true";
    }
    else
    {
        command += " use_sim_time:=false";
    }

    QStringList arguments;

    arguments << "-c" << command;

    nav2_process_->start("bash", arguments);
}

void NAV2Manager::cancelnav2()
{
    if (nav2_process_->state() != QProcess::NotRunning)
    {
        nav2_process_->terminate();

        if (!nav2_process_->waitForFinished(3000))
        {
            nav2_process_->kill();
        }
    }
}

void NAV2Manager::stop()
{
    if (load_map_process_->state() != QProcess::NotRunning)
    {
        load_map_process_->terminate();

        if (!load_map_process_->waitForFinished(3000))
        {
            load_map_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "map_server");

    if (amcl_process_->state() != QProcess::NotRunning)
    {
        amcl_process_->terminate();

        if (!amcl_process_->waitForFinished(3000))
        {
            amcl_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "amcl");

    if (nav2_process_->state() != QProcess::NotRunning)
    {
        nav2_process_->terminate();

        if (!nav2_process_->waitForFinished(3000))
        {
            nav2_process_->kill();
        }
    }
}