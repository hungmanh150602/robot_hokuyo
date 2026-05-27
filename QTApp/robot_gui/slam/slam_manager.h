#ifndef SLAM_MANAGER_H
#define SLAM_MANAGER_H

#include <QObject>
#include <QProcess>

#include <QFileDialog>
#include <QDir>

class SlamManager : public QObject
{
    Q_OBJECT

public:
    explicit SlamManager(QWidget *parent = nullptr);

    void SlamToolBox();
    void stop();
    void saveMap();

signals:
    void newLog(QString text);

private:
    QWidget *parent_widget_;
    QProcess *slam_process_;
    QProcess *save_map_process_;

};

#endif
