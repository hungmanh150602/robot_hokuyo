#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    socket = new QTcpSocket(this);

    connect(ui->btnConnect, &QPushButton::clicked, this, &MainWindow::connectToESP32);
    connect(ui->btnDisConnect, &QPushButton::clicked, this, &MainWindow::disconnectToESP32);

    connect(socket, &QTcpSocket::connected, this, [=]()
            { ui->label_statustcp->setText("Connected"); });
    connect(socket, &QTcpSocket::disconnected, this, [=]()
            { ui->label_statustcp->setText("DisConnected"); });

    // Odometry Publisher
    odomTimer = new QTimer(this);
    connect(odomTimer, &QTimer::timeout, this, &MainWindow::updateOdometry);
    odomTimer->start(20); // 20 ms

    node_ = rclcpp::Node::make_shared("QT_Gui_Node");
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // ros timer
    ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, this, [=]()
            { rclcpp::spin_some(node_); });
    ros_timer->start(10); // 10 ms

    // Rviz
    render_panel_ = new rviz_common::RenderPanel(ui->rvizWidget);
    ui->rvizWidget->layout()->addWidget(render_panel_);
    // Create VisualizationManager
    rviz_ros_node_ = std::make_shared<
            rviz_common::ros_integration::RosNodeAbstraction>("rviz_gui_node");

    manager_ = new rviz_common::VisualizationManager(
            render_panel_,
            rviz_ros_node_,
            nullptr,
            node_->get_clock());

    render_panel_->initialize(manager_);

    manager_->initialize();
    manager_->startUpdate();
    /*
    // Add grid
    grid_ = manager_->createDisplay(
        "rviz_default_plugins/Grid",
        "grid",
        true);
    // Add RobotModel
    robot_model_ = manager_->createDisplay(
        "rviz_default_plugins/RobotModel",
        "robot",
        true);

    robot_model_->subProp("Description Topic")->setValue("/robot_description");
    // Add TF
    tf_ = manager_->createDisplay(
        "rviz_default_plugins/TF",
        "tf",
        true);
    // Add LaserScan
    laser_ = manager_->createDisplay(
        "rviz_default_plugins/LaserScan",
        "scan",
        true);

    laser_->subProp("Topic")->setValue("/scan");
    */
    // Fixed Frame
    manager_->setFixedFrame("odom");
    // ------------------------------------------------------------- //

    // ------------------------------------------------------------- //
    connect(ui->Slider_Lin, &QSlider::valueChanged, this, &MainWindow::updateSpeedDisplay);

    connect(ui->btnForward, &QPushButton::pressed, this, &MainWindow::moveForward);
    connect(ui->btnBack, &QPushButton::pressed, this, &MainWindow::moveBack);
    connect(ui->btnLeft, &QPushButton::pressed, this, &MainWindow::moveLeft);
    connect(ui->btnRight, &QPushButton::pressed, this, &MainWindow::moveRight);
    connect(ui->btnStop, &QPushButton::pressed, this, &MainWindow::stopRobot);

    connect(ui->btnForward, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btnBack, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btnLeft, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btnRight, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btnStop, &QPushButton::released, this, &MainWindow::stopRobot);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::connectToESP32()
{
    QString ip = ui->lineEdit_ip->text();
    int port = ui->lineEdit_port->text().toInt();

    socket->connectToHost(ip, port);
    if (socket->waitForConnected(3000))
    {
        ui->label_statustcp->setText("Connected");
    }
    else
    {
        ui->label_statustcp->setText("Disconnected");
    }
}

void MainWindow::disconnectToESP32()
{
    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        socket->disconnectFromHost();
        ui->label_statustcp->setText("DisConnected");
    }
}

void MainWindow::updateOdometry()
{
    double dt = 0.02;

    x += linear_velocity * cos(theta) * dt;
    y += linear_velocity * sin(theta) * dt;
    theta += angular_velocity * dt;

    ui->lineEditX->setText(QString::number(x, 'f', 2));
    ui->lineEditY->setText(QString::number(y, 'f', 2));
    ui->lineEditTheta->setText(QString::number(theta, 'f', 2));

    left_wheel_angle += left_omega * dt;
    right_wheel_angle += right_omega * dt;

    /* Quaternion */
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    /* TF */
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = node_->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    /* Odom */
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = node_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = angular_velocity;

    odom_pub_->publish(odom);

    /* joint_states */
    sensor_msgs::msg::JointState joint_msg;

    joint_msg.header.stamp = node_->now();
    joint_msg.name.push_back("left_joint");
    joint_msg.name.push_back("right_joint");

    joint_msg.position.push_back(left_wheel_angle);
    joint_msg.position.push_back(right_wheel_angle);

    joint_pub_->publish(joint_msg);
}

void MainWindow::updateSpeedDisplay()
{
    int Lin_vel = ui->Slider_Lin->value();

    ui->lineEdit_Lin->setText(QString::number(Lin_vel));
}

void MainWindow::moveForward()
{
    double Lin_vel = ui->Slider_Lin->value();

    double vL = 0.001915 * Lin_vel + 0.00053;
    double vR = 0.001915 * Lin_vel + 0.00053;

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = -vL / wheel_radius;
    right_omega = vR / wheel_radius;

    QString msg = QString("%1,%2\n").arg(-Lin_vel).arg(Lin_vel);
    socket->write(msg.toUtf8());

    //    qDebug() << "moveForward";
}

void MainWindow::moveBack()
{
    double Lin_vel = ui->Slider_Lin->value();

    double vL = -(0.001915 * Lin_vel + 0.00053);
    double vR = -(0.001915 * Lin_vel + 0.00053);

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = vL / wheel_radius;
    right_omega = -vR / wheel_radius;

    QString msg = QString("%1,%2\n").arg(Lin_vel).arg(-Lin_vel);
    socket->write(msg.toUtf8());

    //    qDebug() << "moveBack";
}

void MainWindow::moveLeft()
{
    double Lin_vel = ui->Slider_Lin->value();

    double vL = -(0.001915 * Lin_vel + 0.00053);
    double vR = 0.001915 * Lin_vel + 0.00053;

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = vL / wheel_radius;
    right_omega = vR / wheel_radius;

    QString msg = QString("%1,%2\n").arg(Lin_vel).arg(Lin_vel);
    socket->write(msg.toUtf8());

    //    qDebug() << "moveLeft";
}

void MainWindow::moveRight()
{
    double Lin_vel = ui->Slider_Lin->value();

    double vL = 0.001915 * Lin_vel + 0.00053;
    double vR = -(0.001915 * Lin_vel + 0.00053);

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = -vL / wheel_radius;
    right_omega = -vR / wheel_radius;

    QString msg = QString("%1,%2\n").arg(-Lin_vel).arg(-Lin_vel);
    socket->write(msg.toUtf8());

    //    qDebug() << "moveRight";
}

void MainWindow::stopRobot()
{
    linear_velocity = 0.0;
    angular_velocity = 0.0;

    left_omega = 0.0;
    right_omega = 0.0;

    QString msg = "0,0\n";
    socket->write(msg.toUtf8());

    //    qDebug() << "stopRobot";
}
