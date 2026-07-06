#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QUrl>
#include <QEvent>
#include <QPainter>
#include <QStyleOptionViewItem>

#include <QTimer>
#include <QVBoxLayout>
#include <QApplication>

#include <math.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tcp_client/tcp_manager.h"
#include "lidar/lidar_manager.h"
#include "camera_manager.h"
#include "person_manager.h"
#include "rviz/rviz_manager.h"
#include "slam/slam_manager.h"
#include "nav2/NAV2_manager.h"
#include "robot/robot_manager.h"
#include "config_path.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QApplication *app, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void sentIP();
    
    void updateStateRobot();
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
    void moveForwardLeft();
    void moveForwardRight();
    void moveBackLeft();
    void moveBackRight();
    void stopRobot();

    void sendNavGoal();

    void killAll();

private:
    Ui::MainWindow *ui;
    QApplication *app_;

    QTimer *stateTimer;
    QTimer *ros_timer;

    TCPManager *tcp;
    LidarManager *lidar;
    Camera_Manager *camera;
    PersonManager *person;
    RVizManager *rviz;
    SlamManager *slam;
    NAV2Manager *nav2;
    RobotManager *robot;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ip_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

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

    double x_goal = 0.0;
    double y_goal = 0.0;
    double yaw_goal = 0.0;
};
#endif // MAINWINDOW_H
