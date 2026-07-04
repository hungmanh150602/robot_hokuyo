#ifndef NAV2_MANAGER_H
#define NAV2_MANAGER_H

#include <QObject>
#include <QProcess>
#include <QTimer>

#include <QFileDialog>
#include <QDir>

class NAV2Manager : public QObject
{
    Q_OBJECT

public:
    explicit NAV2Manager(QWidget *parent = nullptr);

    void checkUseSimTime(bool checked);
    void loadMap();
    void amcl_run();
    void nav2_run();
    void cancelnav2();
    void stop();

signals:
    void newLog(QString text);

private:
    QWidget *parent_widget_;

    bool use_sim_time_ = false;
    
    QProcess *load_map_process_;
    QProcess *configure_active_process_;

    QProcess *amcl_process_;
    QProcess *amcl_configure_active_process_;

    QProcess *nav2_process_;

};

#endif
