#include "tcp_manager.h"
#include "config_path.h"

TCPManager::TCPManager(QObject *parent)
    : QObject(parent)
{
    process_ = new QProcess(this);

    process_->setProcessChannelMode(QProcess::MergedChannels);  // read all: stdout stderr
    connect(process_, &QProcess::readyRead, this, [=]()
    {
        emit newLog(QString::fromLocal8Bit(process_->readAll()));
    });
}

void TCPManager::startTCP()
{
    if (process_->state() != QProcess::NotRunning)
    {
        return;
    }

    QString program = "bash";
    QStringList arguments;

    arguments << "-c"
              << src_ros + " && "
                 + src_ws + " && " + tcp_run;

    process_->start(program, arguments);

    if (!process_->waitForStarted())
    {
        return;
    }
}

void TCPManager::stopTCP()
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
            << "tcp_client_node");
}
