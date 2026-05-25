#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QUrl>
#include <QEvent>
#include <QPainter>
#include <QStyleOptionViewItem>
#include <QFileDialog>

#include <QTcpSocket>
#include <QTimer>
#include <QDebug>
#include <QProcess>
#include <QApplication>
#include <QVBoxLayout>

#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rviz_common
{
class RenderPanel;
class VisualizationManager;

namespace ros_integration
{
class RosNodeAbstraction;
}
}

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QApplication *app, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void connectToESP32();
    void disconnectToESP32();

    void startLidar();
    void stopLidar();

    void initializeRViz();

    void rviz_zoomIn();
    void rviz_zoomOut();
    void rviz_rotateLeft();
    void rviz_rotateRight();
    void rviz_rotateUp();
    void rviz_rotateDown();
    void rviz_resetView();

    void SlamToolBox();

    void loadRobotModel();

    void updateSpeedDisplay();
    void updateOdometry();

    void moveForward();
    void moveBack();
    void moveLeft();
    void moveRight();
    void stopRobot();

private:
    Ui::MainWindow *ui;
    QApplication *app_;

    QTcpSocket *socket;

    QTimer *odomTimer;
    QTimer *ros_timer;

    QProcess *lidar_process;
    QProcess *robot_process_;
    QProcess *slam_process_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    #if 1 /* Rviz */
    rviz_common::RenderPanel *render_panel_;
    rviz_common::VisualizationManager *manager_;

    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_ros_node_;

//    rviz_common::Display *grid_;
//    rviz_common::Display *robot_model_;
//    rviz_common::Display *laser_;
//    rviz_common::Display *tf_;
    #endif

    /* Robot parameter */
    double L = 0.25;
    double wheel_radius = 0.325;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    double left_omega = 0.0;
    double right_omega = 0.0;
    double left_wheel_angle = 0.0;
    double right_wheel_angle = 0.0;
};
#endif // MAINWINDOW_H
