#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QUrl>
#include <QEvent>
#include <QPainter>
#include <QStyleOptionViewItem>

#include <QTcpSocket>
#include <QTimer>
#include <QVBoxLayout>
#include <QApplication>

#include <math.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "lidar/lidar_manager.h"
#include "rviz/rviz_manager.h"
#include "slam/slam_manager.h"
#include "robot/robot_manager.h"



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

    void updateSpeedDisplay();
    void updateOdometry();

    void updateFrameList();
    void updateLaserTopics();
    void updateMapTopics();

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

    LidarManager *lidar;
    RVizManager *rviz;
    SlamManager *slam;
    RobotManager *robot;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

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
