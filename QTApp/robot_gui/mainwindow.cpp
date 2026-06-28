#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QApplication *app, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), app_(app)
{
    ui->setupUi(this);

/* ================ TCP Socket ================ */
#if USE_TCP_SOCKET
    socket = new QTcpSocket(this);

    connect(ui->btnConnect, &QPushButton::clicked, this, &MainWindow::connectToESP32);
    connect(ui->btnDisConnect, &QPushButton::clicked, this, &MainWindow::disconnectToESP32);

    // Log
    connect(socket, &QTcpSocket::connected, this, [=]()
            { ui->label_statustcp->setText("Connected"); });
    connect(socket, &QTcpSocket::disconnected, this, [=]()
            { ui->label_statustcp->setText("DisConnected"); });
#endif
/* =============================================================================== */

/* ================ ROS timer ================ */
#if USE_ROS_TIMER
    ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, this, [=]()
            { rclcpp::spin_some(node_); });
    ros_timer->start(20); // 10 ms

    stateTimer = new QTimer(this);
    connect(stateTimer, &QTimer::timeout, this,
            &MainWindow::updateStateRobot);
    stateTimer->start(20); // 10 ms
#endif
/* =============================================================================== */

/* ================ Subscriber and Publisher ================ */
#if USE_SUB_AND_PUB
    node_ = rclcpp::Node::make_shared("QT_Gui_Node");

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        20,
        std::bind(&MainWindow::CmdVelCallback, this, std::placeholders::_1));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_camera",
        20,
        std::bind(&MainWindow::odomCallback, this, std::placeholders::_1));

    camera_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/camera_status",
        10,
        std::bind(&MainWindow::cameraCallback, this, std::placeholders::_1));

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
#endif
/* =============================================================================== */

/* ================ Rviz ================ */
#if USE_RVIZ
    rviz = new RVizManager(app_, ui->rvizWidget, node_, this);

    rviz->initializeRViz();

    // button rviz
    connect(ui->btnRvizZoomIn, &QPushButton::clicked, rviz, &RVizManager::zoomIn);
    connect(ui->btnRvizZoomOut, &QPushButton::clicked, rviz, &RVizManager::zoomOut);
    connect(ui->btnRvizResetView, &QPushButton::clicked, rviz, &RVizManager::topView);
    connect(ui->btnRvizReset, &QPushButton::clicked, rviz, &RVizManager::resetRViz);

    // TF
    connect(ui->checkBox_TF, &QCheckBox::toggled, rviz, &RVizManager::enableTF);
    // Frame
    connect(ui->btn_FixFrame, &QPushButton::clicked, this, &MainWindow::updateFrameList);
    connect(ui->comboBox_FixFrame, &QComboBox::currentTextChanged, rviz, &RVizManager::setFixedFrame);
    // Laser
    connect(ui->btn_LaserScan, &QPushButton::clicked, this, &MainWindow::updateLaserTopics);
    connect(ui->comboBox_Laser, &QComboBox::currentTextChanged, rviz, &RVizManager::setLaserTopic);
    // Map
    connect(ui->btn_Maps, &QPushButton::clicked, this, &MainWindow::updateMapTopics);
    connect(ui->comboBox_Maps, &QComboBox::currentTextChanged, rviz, &RVizManager::setMapTopic);
    // MarkerArray
    connect(ui->btn_MarkerArr, &QPushButton::clicked, this, &MainWindow::updateMarkerArrayTopics);
    connect(ui->comboBox_MarkerArr, &QComboBox::currentTextChanged, rviz, &RVizManager::setMarkerTopic);
    // 2D Pose Estimate
    connect(ui->btn_2DPose, &QPushButton::clicked, rviz, &RVizManager::setInitialPoseTool);
    // 2D Goal Pose
    connect(ui->btn_2DGoal, &QPushButton::clicked, rviz, &RVizManager::setGoalPoseTool);
#endif
/* =============================================================================== */

/* ================ Load Robot Model ================ */
#if USE_LOAD_ROBOT
    robot = new RobotManager(this);
    // Log
    connect(robot, &RobotManager::newLog, this, [=](QString text)
            { ui->textEdit_log->append(text); });
    // Button
    connect(ui->btn_LoadRobot, &QPushButton::clicked, robot, &RobotManager::loadRobotModel);
#endif
/* =============================================================================== */

/* ================ Connect lidar ================ */
#if USE_LIDAR
    lidar = new LidarManager(this);

    // Log
    connect(lidar, &LidarManager::newLog, this, [=](QString text)
            { ui->textEdit_log->append(text); });
    // Button
    connect(ui->btnConnect_lidar, &QPushButton::clicked, lidar, &LidarManager::startLidar);
    connect(ui->btnStop_lidar, &QPushButton::clicked, lidar, &LidarManager::stopLidar);
#endif
/* =============================================================================== */

/* ================ Connect camera ================ */
#if USE_CAMERA
    camera = new Camera_Manager(this);

    // Log
    connect(camera, &Camera_Manager::newLog, this, [=](QString text)
            { ui->textEdit_log->append(text); });
    // Button
    connect(ui->btn_StartCamera, &QPushButton::clicked, camera, &Camera_Manager::startCamera);
    connect(ui->btn_StopCamera, &QPushButton::clicked, camera, &Camera_Manager::stopCamera);
#endif
    /* =============================================================================== */

/* ================ Connect leg_follower ================ */
#if USE_LEG_FOLLOWER
    person = new PersonManager(this);

    // Log
    connect(person, &PersonManager::newLog, this, [=](QString text)
            { ui->textEdit_log->append(text); });
    // Button
    connect(ui->btn_PerTracker, &QPushButton::clicked, person, &PersonManager::startPersonTracker);
    connect(ui->btn_StopTracker, &QPushButton::clicked, person, &PersonManager::stop);
#endif
    /* =============================================================================== */

/* ================ Slam ToolBox and NAV2 ================ */
#if USE_SLAM_AND_NAV2
    slam = new SlamManager(this);

    // Log
    connect(slam, &SlamManager::newLog, this, [=](QString text)
            { ui->textEdit_log->append(text); });
    // Button
    connect(ui->btn_SlamToolBox, &QPushButton::clicked, slam, &SlamManager::SlamToolBox);
    connect(ui->btn_SaveMap, &QPushButton::clicked, slam, &SlamManager::saveMap);
    connect(ui->checkBox_UseSimTime, &QCheckBox::toggled, slam, &SlamManager::checkUseSimTime);
    connect(ui->btn_LoadMap, &QPushButton::clicked, slam, &SlamManager::loadMap);
    connect(ui->btn_AMCL, &QPushButton::clicked, slam, &SlamManager::amcl_run);
    connect(ui->btn_NAV2, &QPushButton::clicked, slam, &SlamManager::nav2_run);
#endif
/* =============================================================================== */

/* ================ UI Button ================ */
#if USE_BUTTON_CONTROL
    /* btn rst pose */
    connect(ui->btn_RSTPose, &QPushButton::clicked, this, &MainWindow::resetPose);

    connect(ui->btn_killall, &QPushButton::clicked, this, &MainWindow::killAll);

    /* btn control robot */
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
#endif
    /* =============================================================================== */
}

MainWindow::~MainWindow()
{
    killAll();

    if (robot)
    {
        robot->stop();
    }

    if (rviz)
    {
        rviz->stop();
    }

    if (socket)
    {
        socket->disconnectFromHost();
    }

    rclcpp::shutdown();

    delete ui;
}

void MainWindow::killAll()
{
    if (slam)
    {
        slam->stop();
    }

    if (lidar)
    {
        lidar->stopLidar();
    }

    if (camera)
    {
        camera->stopCamera();
    }

    if (person)
    {
        person->stop();
    }

    ui->textEdit_log->append("Stop All: Lidar, Camera, Person Tracker, Slam, Map, AMCL, NAV2!!!");
}

void MainWindow::connectToESP32()
{
    QString ip = ui->lineEdit_ip->text();
    int port = ui->lineEdit_port->text().toInt();

    socket->connectToHost(ip, port);
    if (socket->waitForConnected(3000))
    {
        ui->label_statustcp->setText("Connected");
        ui->textEdit_log->append("Connected to ESP32.");
    }
    else
    {
        ui->label_statustcp->setText("Disconnected");
        ui->textEdit_log->append("Fail Connect to ESP32!");
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

void MainWindow::cameraCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_camera = msg->data;

    if (!is_camera)
    {
        stopRobot();
        ui->textEdit_log->append("Camera lost");
        return;
    }
}

void MainWindow::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    linear_velocity = msg->linear.x;
    angular_velocity = msg->angular.z;

#if USE_CAMERA_STATUS
    if (!is_camera)
        return;
#endif

    // compute msg send to tcp
    double vR = linear_velocity + L / 2.0 * angular_velocity;
    double vL = linear_velocity - L / 2.0 * angular_velocity;
    int msg_vel_L = (vL - 0.00053) / 0.001915;
    int msg_vel_R = (vR - 0.00053) / 0.001915;

    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        QString msg = QString("%1,%2\n").arg(-msg_vel_L).arg(msg_vel_R);
        socket->write(msg.toUtf8());
    }
}

void MainWindow::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    double roll, pitch, yaw;

    tf2::Matrix3x3(q).getRPY(
        roll,
        pitch,
        yaw);

    odom_theta = yaw;

    linear_velocity = msg->twist.twist.linear.x;
    angular_velocity = msg->twist.twist.angular.z;
}

void MainWindow::resetPose()
{
    robot_x = 0.0;
    robot_y = 0.0;
    robot_theta = 0.0;

    odom_x = 0.0;
    odom_y = 0.0;
    odom_theta = 0.0;

    prev_x = 0.0;
    prev_y = 0.0;
    prev_theta = 0.0;
}

void MainWindow::updateStateRobot()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        prev_x = robot_x;
        prev_y = robot_y;
        prev_theta = robot_theta;
        return;
    }
#endif

    robot_x = odom_x + prev_x;
    robot_y = odom_y + prev_y;
    robot_theta = odom_theta + prev_theta;

    int Lin_vel = ui->Slider_Lin->value();

    ui->lineEdit_Lin->setText(QString::number(Lin_vel));
    ui->lineEditX->setText(QString::number(robot_x, 'f', 2));
    ui->lineEditY->setText(QString::number(robot_y, 'f', 2));
    ui->lineEditTheta->setText(QString::number(robot_theta, 'f', 2));

    double dt = 0.02;

#if USE_WHEEL_ODOM
    robot_x += linear_velocity * cos(theta) * dt;
    robot_y += linear_velocity * sin(theta) * dt;
    robot_theta += angular_velocity * dt;
#endif

    /* Quaternion */
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_theta);

    /* TF */
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = node_->now();
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = robot_x;
    tf.transform.translation.y = robot_y;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);

    /* Publish Odom */
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = node_->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = robot_x;
    odom_msg.pose.pose.position.y = robot_y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.angular.z = angular_velocity;

    odom_pub_->publish(odom_msg);

    /* joint_states */
    sensor_msgs::msg::JointState joint_msg;

    double vR = linear_velocity + L / 2.0 * angular_velocity;
    double vL = linear_velocity - L / 2.0 * angular_velocity;

    left_omega = vL / wheel_radius;
    right_omega = vR / wheel_radius;

    left_wheel_angle += left_omega * dt;
    right_wheel_angle += right_omega * dt;

    joint_msg.header.stamp = node_->now();
    joint_msg.name.push_back("left_joint");
    joint_msg.name.push_back("right_joint");

    joint_msg.position.push_back(left_wheel_angle);
    joint_msg.position.push_back(right_wheel_angle);

    joint_pub_->publish(joint_msg);
}

void MainWindow::updateFrameList()
{
    QStringList list_frames = rviz->getAllFrames();

    ui->comboBox_FixFrame->clear();
    ui->comboBox_FixFrame->addItems(list_frames);
}

void MainWindow::updateLaserTopics()
{
    QStringList topics = rviz->getLaserTopics();

    ui->comboBox_Laser->clear();
    ui->comboBox_Laser->addItems(topics);
}

void MainWindow::updateMapTopics()
{
    QStringList topics = rviz->getMapTopics();

    ui->comboBox_Maps->clear();
    ui->comboBox_Maps->addItems(topics);
}

void MainWindow::updateMarkerArrayTopics()
{
    QStringList topics = rviz->getMarkerTopics();

    ui->comboBox_MarkerArr->clear();
    ui->comboBox_MarkerArr->addItems(topics);
}

void MainWindow::moveForward()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    double Lin_vel = ui->Slider_Lin->value();

    double vL = 0.001915 * Lin_vel + 0.00053;
    double vR = 0.001915 * Lin_vel + 0.00053;

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = -vL / wheel_radius;
    right_omega = vR / wheel_radius;

    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        QString msg = QString("%1,%2\n").arg(-Lin_vel).arg(Lin_vel);
        socket->write(msg.toUtf8());
    }
}

void MainWindow::moveBack()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    double Lin_vel = ui->Slider_Lin->value();

    double vL = -(0.001915 * Lin_vel + 0.00053);
    double vR = -(0.001915 * Lin_vel + 0.00053);

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = vL / wheel_radius;
    right_omega = -vR / wheel_radius;

    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        QString msg = QString("%1,%2\n").arg(Lin_vel).arg(-Lin_vel);
        socket->write(msg.toUtf8());
    }
}

void MainWindow::moveLeft()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    double Lin_vel = ui->Slider_Lin->value();

    double vL = -(0.001915 * Lin_vel + 0.00053);
    double vR = 0.001915 * Lin_vel + 0.00053;

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = vL / wheel_radius;
    right_omega = vR / wheel_radius;

    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        QString msg = QString("%1,%2\n").arg(Lin_vel).arg(Lin_vel);
        socket->write(msg.toUtf8());
    }
}

void MainWindow::moveRight()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    double Lin_vel = ui->Slider_Lin->value();

    double vL = 0.001915 * Lin_vel + 0.00053;
    double vR = -(0.001915 * Lin_vel + 0.00053);

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = -vL / wheel_radius;
    right_omega = -vR / wheel_radius;

    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        QString msg = QString("%1,%2\n").arg(-Lin_vel).arg(-Lin_vel);
        socket->write(msg.toUtf8());
    }
}

void MainWindow::stopRobot()
{
    linear_velocity = 0.0;
    angular_velocity = 0.0;

    left_omega = 0.0;
    right_omega = 0.0;

    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        QString msg = QString("0,0\n");
        socket->write(msg.toUtf8());
    }
}
