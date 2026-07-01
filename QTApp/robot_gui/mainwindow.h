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
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "lidar/lidar_manager.h"
#include "camera_manager.h"
#include "person_manager.h"
#include "rviz/rviz_manager.h"
#include "slam/slam_manager.h"
#include "robot/robot_manager.h"

#define USE_CAMERA_STATUS       1
#define USE_WHEEL_ODOM          0
#define USE_TCP_SOCKET          1
#define USE_ROS_TIMER           1
#define USE_SUB_AND_PUB         1
#define USE_RVIZ                1
#define USE_LOAD_ROBOT          1
#define USE_LIDAR               1
#define USE_CAMERA              1
#define USE_LEG_FOLLOWER        1
#define USE_SLAM_AND_NAV2       1
#define USE_BUTTON_CONTROL      1

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

    void updateStateRobot();
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cameraCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void resetPose();

    void updateFrameList();
    void updateLaserTopics();
    void updateMapTopics();
    void updateMarkerArrayTopics();

    void moveForward();
    void moveBack();
    void moveLeft();
    void moveRight();
    void stopRobot();

    void killAll();

private:
    Ui::MainWindow *ui;
    QApplication *app_;

    QTcpSocket *socket;

    QTimer *stateTimer;
    QTimer *ros_timer;

    LidarManager *lidar;
    Camera_Manager *camera;
    PersonManager *person;
    RVizManager *rviz;
    SlamManager *slam;
    RobotManager *robot;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    /* Robot parameter */
    double L = 0.25;
    double wheel_radius = 0.325;

    bool is_camera = false;

    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_theta = 0.0;

    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    double left_omega = 0.0;
    double right_omega = 0.0;
    double left_wheel_angle = 0.0;
    double right_wheel_angle = 0.0;
};
#endif // MAINWINDOW_H
