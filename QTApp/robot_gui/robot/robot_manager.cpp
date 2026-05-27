#include "robot_manager.h"

RobotManager::RobotManager(QObject *parent)
    : QObject(parent)
{
    robot_process_ = new QProcess(this);

    robot_process_->setProcessChannelMode(QProcess::MergedChannels);
    connect(robot_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(robot_process_->readAll()));
    });
}

void RobotManager::loadRobotModel()
{
    QString fileName = "/home/hungubuntu/ros2_workspace/src/robot_hokuyo/urdf/Final_Assembly2.xacro";


    if(fileName.isEmpty())
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
    command =
            "source /opt/ros/humble/setup.bash && "
            "source ~/ros2_workspace/install/setup.bash && "
            "xacro " + fileName + " > " + urdfFile + " && "
            "ros2 run robot_state_publisher "
            "robot_state_publisher "
            + urdfFile;
    #else /* urdf */
    command =
            "source /opt/ros/humble/setup.bash && "
            "source ~/" + ros2_ws + "/install/setup.bash && "
            "ros2 run robot_state_publisher "
            "robot_state_publisher "
            + fileName;
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
}
