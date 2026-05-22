#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>
#include <QDebug>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display.hpp>

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void connectToESP32();
    void disconnectToESP32();

    void updateSpeedDisplay();
    void updateOdometry();

    void moveForward();
    void moveBack();
    void moveLeft();
    void moveRight();
    void stopRobot();

private:
    Ui::MainWindow *ui;

    QTcpSocket *socket;

    QTimer *odomTimer;
    QTimer *ros_timer;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_ros_node_;

    rviz_common::RenderPanel *render_panel_;
    rviz_common::VisualizationManager *manager_;
    rviz_common::Display *grid_;
    rviz_common::Display *robot_model_;
    rviz_common::Display *laser_;
    rviz_common::Display *tf_;
   
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
