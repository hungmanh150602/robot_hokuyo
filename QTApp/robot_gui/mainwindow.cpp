#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QApplication *app, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), app_(app)
{
    ui->setupUi(this);

/* ================ TCP Socket ================ */
#if USE_TCP_SOCKET
    tcp = new TCPManager(this);

    tcp->startTCP();
    // Log
    connect(tcp, &TCPManager::newLog, this, [=](QString text)
            { ui->textEdit_log->append(text); });
    // Button
    connect(ui->btnConnect, &QPushButton::clicked, this, &MainWindow::sentIP);
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

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        20,
        std::bind(&MainWindow::odomCallback, this, std::placeholders::_1));

    camera_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/camera_status",
        10,
        std::bind(&MainWindow::cameraCallback, this, std::placeholders::_1));

    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);

    ip_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/ip_connect",
        10);

    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10);
#endif
/* =============================================================================== */

/* ================ Rviz ================ */
#if USE_RVIZ
    rviz = new RVizManager(app_, ui->rvizWidget, node_, this);

    rviz->initializeRViz();

    // button rviz
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

/* ================ Slam ToolBox ================ */
#if USE_SLAM_TOOLBOX
    slam = new SlamManager(this);

    // Log
    connect(slam, &SlamManager::newLog, this, [=](QString text)
            { ui->textEdit_log->append(text); });
    // Button
    connect(ui->btn_SlamToolBox, &QPushButton::clicked, slam, &SlamManager::SlamToolBox);
    connect(ui->btn_SaveMap, &QPushButton::clicked, slam, &SlamManager::saveMap);
#endif
/* =============================================================================== */

/* ================ NAV2 ================ */
#if USE_NAV2
    nav2 = new NAV2Manager(this);

    // Log
    connect(nav2, &NAV2Manager::newLog, this, [=](QString text)
            { ui->textEdit_log->append(text); });
    // Button
    connect(ui->checkBox_UseSimTime, &QCheckBox::toggled, nav2, &NAV2Manager::checkUseSimTime);
    connect(ui->btn_LoadMap, &QPushButton::clicked, nav2, &NAV2Manager::loadMap);
    connect(ui->btn_AMCL, &QPushButton::clicked, nav2, &NAV2Manager::amcl_run);
    connect(ui->btn_NAV2, &QPushButton::clicked, nav2, &NAV2Manager::nav2_run);
    connect(ui->comboBox_Goal, &QComboBox::currentTextChanged, this, [=](const QString &text)
            {
                if (text == "Home")
                {
                    sendNavGoal(0.0, 0.0, 0.0);
                }
                else if (text == "Corner")
                {
                    sendNavGoal(3.6, 1.2, M_PI / 2.0);
                } });
    connect(ui->btn_CancelGoal, &QPushButton::clicked, nav2, &NAV2Manager::cancelnav2);
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
    connect(ui->btn_FL, &QPushButton::pressed, this, &MainWindow::moveForwardLeft);
    connect(ui->btn_FR, &QPushButton::pressed, this, &MainWindow::moveForwardRight);
    connect(ui->btn_BL, &QPushButton::pressed, this, &MainWindow::moveBackLeft);
    connect(ui->btn_BR, &QPushButton::pressed, this, &MainWindow::moveBackRight);

    connect(ui->btnForward, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btnBack, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btnLeft, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btnRight, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btn_FL, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btn_FR, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btn_BL, &QPushButton::released, this, &MainWindow::stopRobot);
    connect(ui->btn_BR, &QPushButton::released, this, &MainWindow::stopRobot);

    connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::stopRobot);
#endif
    /* =============================================================================== */
}

MainWindow::~MainWindow()
{
    if (tcp)
    {
        tcp->stopTCP();
    }

    killAll();

    if (robot)
    {
        robot->stop();
    }

    if (rviz)
    {
        rviz->stop();
    }

    rclcpp::shutdown();

    delete ui;
}

void MainWindow::killAll()
{
    if (nav2)
    {
        nav2->stop();
    }

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

void MainWindow::sentIP()
{
    std_msgs::msg::String msg;

    QString ip = ui->lineEdit_ip->text();

    msg.data = ip.toStdString();

    ip_pub_->publish(msg);
}

void MainWindow::cameraCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_camera = msg->data;

    if (!is_camera)
    {
        stopRobot();
        ui->textEdit_log->append("Camera lost!!!");
        return;
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
    odom_x = 0.0;
    odom_y = 0.0;
    odom_theta = 0.0;
}

void MainWindow::updateStateRobot()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    int Lin_vel = ui->Slider_Lin->value();

    ui->lineEdit_Lin->setText(QString::number(Lin_vel));
    ui->lineEditX->setText(QString::number(odom_x, 'f', 2));
    ui->lineEditY->setText(QString::number(odom_y, 'f', 2));
    ui->lineEditTheta->setText(QString::number(odom_theta, 'f', 2));

#if USE_WHEEL_ODOM
    robot_x += linear_velocity * cos(theta) * dt;
    robot_y += linear_velocity * sin(theta) * dt;
    robot_theta += angular_velocity * dt;
#endif
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

void MainWindow::sendNavGoal(double x, double y, double yaw)
{
    geometry_msgs::msg::PoseStamped goal;

    goal.header.stamp = node_->now();
    goal.header.frame_id = "map";

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    goal_pub_->publish(goal);

    ui->textEdit_log->append(QString("Send Goal: x=%1 y=%2 yaw=%3")
                                .arg(x)
                                .arg(y)
                                .arg(yaw));
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

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
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

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
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

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
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

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
}

void MainWindow::moveForwardLeft()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    double Lin_vel = ui->Slider_Lin->value();

    double vL = (0.001915 * Lin_vel + 0.00053) / 2;
    double vR = 0.001915 * Lin_vel + 0.00053;

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = -vL / wheel_radius;
    right_omega = vR / wheel_radius;

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
}

void MainWindow::moveForwardRight()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    double Lin_vel = ui->Slider_Lin->value();

    double vL = 0.001915 * Lin_vel + 0.00053;
    double vR = (0.001915 * Lin_vel + 0.00053) / 2;

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = vL / wheel_radius;
    right_omega = -vR / wheel_radius;

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
}

void MainWindow::moveBackLeft()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    double Lin_vel = ui->Slider_Lin->value();

    double vL = -(0.001915 * Lin_vel + 0.00053) / 2;
    double vR = -(0.001915 * Lin_vel + 0.00053);

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = vL / wheel_radius;
    right_omega = vR / wheel_radius;

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
}

void MainWindow::moveBackRight()
{
#if USE_CAMERA_STATUS
    if (!is_camera)
    {
        return;
    }
#endif

    double Lin_vel = ui->Slider_Lin->value();

    double vL = -(0.001915 * Lin_vel + 0.00053);
    double vR = -(0.001915 * Lin_vel + 0.00053) / 2;

    linear_velocity = (vR + vL) / 2.0;
    angular_velocity = (vR - vL) / L;

    left_omega = -vL / wheel_radius;
    right_omega = -vR / wheel_radius;

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
}

void MainWindow::stopRobot()
{
    linear_velocity = 0.0;
    angular_velocity = 0.0;

    left_omega = 0.0;
    right_omega = 0.0;

    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;

    cmd_pub_->publish(msg);
}
