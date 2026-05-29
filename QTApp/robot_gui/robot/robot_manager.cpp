#include "robot_manager.h"
#include "config_path.h"

RobotManager::RobotManager(QObject *parent)
    : QObject(parent)
{
    robot_process_ = new QProcess(this);
    map_tf_process_ = new QProcess(this);

    robot_process_->setProcessChannelMode(QProcess::MergedChannels);
    map_tf_process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr

    connect(robot_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(robot_process_->readAll()));
    });

    connect(map_tf_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(map_tf_process_->readAll()));
    });
}

void RobotManager::loadRobotModel()
{
    QString tf_command =
        src_ros + " && "
        "ros2 run tf2_ros static_transform_publisher "
        "0 0 0 0 0 0 map odom";

    map_tf_process_->start("bash", QStringList() << "-c" << tf_command);

    if(robot_fileName.isEmpty())
    {
        return;
    }

    /* Kill old robot_state_publisher */
    if(robot_process_->state() != QProcess::NotRunning)
    {
        robot_process_->kill();
        robot_process_->waitForFinished();
    }

    QString command;
    QString urdfFile = "/tmp/robot.urdf";

    #if 1 /* Xacro */
    command = src_ros + " && "
              + src_ws + " && "
              "xacro " + robot_fileName + " > " + urdfFile + " && "
              + robot_run + " robot_state_publisher " + urdfFile;
    #else /* urdf */
    command =
            src_ros + " && "
            + src_ws + " && "
            + robot_run + " robot_state_publisher "
            + robot_fileName;
    #endif

    robot_process_->start("bash", QStringList() << "-c" << command);
}

void RobotManager::stop()
{
    if(robot_process_->state() != QProcess::NotRunning)
    {
        robot_process_->terminate();

        if(!robot_process_->waitForFinished(3000))
        {
            robot_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "robot_state_publisher");

    if(map_tf_process_->state() != QProcess::NotRunning)
    {
        map_tf_process_->terminate();

        if(!map_tf_process_->waitForFinished(3000))
        {
            map_tf_process_->kill();
        }
    }

    QProcess::execute(
        "pkill",
        QStringList()
            << "-f"
            << "static_transform_publisher");
}
