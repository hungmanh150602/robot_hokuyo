#include "robot_manager.h"
#include "config_path.h"

RobotManager::RobotManager(QObject *parent)
    : QObject(parent)
{
    robot_process_ = new QProcess(this);

    #if 1
    robot_process_->setProcessChannelMode(QProcess::MergedChannels);

    connect(robot_process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(robot_process_->readAll()));
    });
    #endif
}

void RobotManager::loadRobotModel()
{
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

    #if USE_XACRO_FILE /* Xacro */
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
    emit newLog("Load Robot SUCCESSFUL!!!");
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
