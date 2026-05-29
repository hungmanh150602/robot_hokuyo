#ifndef SLAM_MANAGER_H
#define SLAM_MANAGER_H

#include <QObject>
#include <QProcess>
#include <QTimer>

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
    void checkUseSimTime(bool checked);
    void loadMap();

signals:
    void newLog(QString text);

private:
    QWidget *parent_widget_;
    QProcess *slam_process_;
    QProcess *save_map_process_;
    QProcess *load_map_process_;
    QProcess *configure_active_process_;

    bool use_sim_time_ = false;
};

#endif
