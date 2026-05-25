#include "mainwindow.h"
#include "ui_mainwindow.h"

#if 1 /* RViz */
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/display.hpp>

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <rviz_rendering/render_window.hpp>
#include <rviz_rendering/render_system.hpp>
#endif

MainWindow::MainWindow(QApplication *app, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), app_(app)
{
    ui->setupUi(this);

    #if 1 /* TCP Socket */
    socket = new QTcpSocket(this);

    connect(ui->btnConnect, &QPushButton::clicked, this, &MainWindow::connectToESP32);
    connect(ui->btnDisConnect, &QPushButton::clicked, this, &MainWindow::disconnectToESP32);

    connect(socket, &QTcpSocket::connected, this, [=](){ ui->label_statustcp->setText("Connected"); });
    connect(socket, &QTcpSocket::disconnected, this, [=](){ ui->label_statustcp->setText("DisConnected"); });
    #endif

    #if 1 /* Connect lidar */
    lidar_process = new QProcess(this);
    lidar_process->setProcessChannelMode(QProcess::MergedChannels);

    connect(ui->btnConnect_lidar, &QPushButton::clicked, this, &MainWindow::startLidar);
    connect(ui->btnStop_lidar, &QPushButton::clicked, this, &MainWindow::stopLidar);

    connect(lidar_process, &QProcess::readyRead, this, [=](){
        QByteArray data = lidar_process->readAll();

        ui->textEdit_log->append(QString(data));
    });
    #endif

    #if 1 /* Odometry,TF Publisher */
    odomTimer = new QTimer(this);
    connect(odomTimer, &QTimer::timeout, this, &MainWindow::updateOdometry);
    odomTimer->start(20); // 20 ms

    node_ = rclcpp::Node::make_shared("QT_Gui_Node");
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    #endif

    #if 1 /* ros timer */
    ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, this, [=](){ rclcpp::spin_some(node_); });
    ros_timer->start(10); // 10 ms
    #endif

    #if 1 /* Rviz */
    initializeRViz();

    /* btn rviz */
    connect(ui->btnRvizZoomIn, &QPushButton::clicked, this, &MainWindow::rviz_zoomIn);
    connect(ui->btnRvizZoomOut, &QPushButton::clicked, this, &MainWindow::rviz_zoomOut);
    connect(ui->btnRvizLeft, &QPushButton::clicked, this, &MainWindow::rviz_rotateLeft);
    connect(ui->btnRvizRight, &QPushButton::clicked, this, &MainWindow::rviz_rotateRight);
    connect(ui->btnRvizUp, &QPushButton::clicked, this, &MainWindow::rviz_rotateUp);
    connect(ui->btnRvizDown, &QPushButton::clicked, this, &MainWindow::rviz_rotateDown);
    connect(ui->btnRvizReset, &QPushButton::clicked, this, &MainWindow::rviz_resetView);
    #endif

    #if 1 /* Load Robot Model */
    robot_process_ = new QProcess(this);

    connect(ui->btn_LoadRobot, &QPushButton::clicked, this, &MainWindow::loadRobotModel);
    #endif

    #if 1 /* Slam Tool Box */
    slam_process_ = new QProcess(this);
    connect(ui->btn_SlamToolBox, &QPushButton::clicked, this, &MainWindow::SlamToolBox);
    #endif

    #if 1 /* Button */
    /* slider */
    connect(ui->Slider_Lin, &QSlider::valueChanged, this, &MainWindow::updateSpeedDisplay);

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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::connectToESP32()
{
    ui->textEdit_log->append("Connecting to ESP32 ...");

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
        ui->textEdit_log->append("Disconnected to ESP32.");
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

void MainWindow::startLidar()
{
    if (lidar_process->state() != QProcess::NotRunning)
    {
        ui->textEdit_log->append("LiDAR already running");
        return;
    }

    QString program = "bash";
    QStringList arguments;

    arguments << "-c"
              << "source ~/ros2_workspace/install/setup.bash && "
                 "ros2 run robot_hokuyo publish_lidar";

    lidar_process->start(program, arguments);

    if (!lidar_process->waitForStarted())
    {
        ui->textEdit_log->append("Failed to start LiDAR");
    }
}

void MainWindow::stopLidar()
{
    lidar_process->kill();
}

void MainWindow::initializeRViz()
{
    /* Initialize render system */
    rviz_rendering::RenderSystem::get();
    app_->processEvents();

    /* Create render panel */
    render_panel_ = new rviz_common::RenderPanel(ui->rvizWidget);
    app_->processEvents();

    /* IMPORTANT */
    render_panel_->getRenderWindow()->initialize();

    /* Create Rviz Ros Node */
    rviz_ros_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_gui_node");

    /* Create VisualizationManager */
    manager_ = new rviz_common::VisualizationManager(render_panel_, rviz_ros_node_, nullptr, node_->get_clock());

    /* Initialize render panel */
    render_panel_->initialize(manager_);
    app_->processEvents();

    /* Initialize manager */
    manager_->initialize();

    /* Start update */
    manager_->startUpdate();

    #if 1/* Layout */
    if(ui->rvizWidget->layout() == nullptr)
    {
        QVBoxLayout *layout = new QVBoxLayout(ui->rvizWidget);

        layout->setContentsMargins(0,0,0,0);

        layout->addWidget(render_panel_);
    }
    else
    {
        ui->rvizWidget->layout()->addWidget(render_panel_);
    }
    #endif

    #if 1 /* Add Grid */
    auto grid = manager_->createDisplay(
                "rviz_default_plugins/Grid",
                "Grid",
                true);

    Q_UNUSED(grid);
    #endif

    #if 0 /* Fixed frame */
    manager_->setFixedFrame("odom");
    #endif

    #if 0 /* Add TF */
    auto tf = manager_->createDisplay(
                "rviz_default_plugins/TF",
                "TF",
                true);

    Q_UNUSED(tf);
    #endif

    #if 0 /* Add LaserScan */
    auto laser = manager_->createDisplay(
                "rviz_default_plugins/LaserScan",
                "Laser",
                true);

    laser->subProp("Topic")->setValue("/scan");
    #endif

    #if 0 /* map */
    auto map = manager_->createDisplay(
            "rviz_default_plugins/Map",
            "Map",
            true);

    map->subProp("Topic")->setValue("/map");
    #endif

    #if 0 /* trajectory */
    auto pose =
        manager_->createDisplay(
            "rviz_default_plugins/Pose",
            "Pose",
            true);
    #endif
}

void MainWindow::loadRobotModel()
{
    QString fileName = "/home/hungubuntu/ros2_workspace/src/"
                       "robot_hokuyo/urdf/Final_Assembly2.xacro";


    if(fileName.isEmpty())
    {
        ui->textEdit_log->append("No such file found!");
        return;
    }
    ui->textEdit_log->append("Robot Model Loaded!");

    /* Kill old robot_state_publisher */
    if(robot_process_->state() != QProcess::NotRunning)
    {
        robot_process_->kill();
        robot_process_->waitForFinished();
    }

    QString command;
    QString urdfFile = "/tmp/robot.urdf";

    #if 1 /* Xacro */
    command =
            "source /opt/ros/humble/setup.bash && "
            "source ~/ros2_workspace/install/setup.bash && "
            "xacro " + fileName + " > " + urdfFile + " && "
            "ros2 run robot_state_publisher "
            "robot_state_publisher "
            + urdfFile;
    #else /* urdf */
    command =
            "source /opt/ros/humble/setup.bash && "
            "source ~/ros2_workspace/install/setup.bash && "
            "ros2 run robot_state_publisher "
            "robot_state_publisher "
            + fileName;
    #endif

    robot_process_->start("bash", QStringList() << "-c" << command);

    /* Add RobotModel display */
    auto robot_model = manager_->createDisplay(
                "rviz_default_plugins/RobotModel",
                "Robot Model",
                true);

    robot_model->subProp("Description Source")->setValue("Topic");
    robot_model->subProp("Description Topic")->setValue("/robot_description");

    Q_UNUSED(robot_model);

    /* Add TF display */
    auto tf = manager_->createDisplay(
            "rviz_default_plugins/TF",
            "TF",
            true);

    Q_UNUSED(tf);

    /* Fixed frame */
    manager_->setFixedFrame("odom");
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

void MainWindow::rviz_zoomIn()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view)
    {
        float dist =view->subProp("Distance")->getValue().toFloat();

        dist += 1.0;

        if(dist < 0.5)
            dist = 0.5;

        view->subProp("Distance")->setValue(dist);
    }
}

void MainWindow::rviz_zoomOut()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view)
    {
        float dist = view->subProp("Distance")->getValue().toFloat();

        dist -= 1.0;

        view->subProp("Distance")->setValue(dist);
    }
}

void MainWindow::rviz_rotateLeft()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view)
    {
        float yaw = view->subProp("Yaw")->getValue().toFloat();

        yaw += 0.1;

        view->subProp("Yaw")->setValue(yaw);
    }
}

void MainWindow::rviz_rotateRight()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view)
    {
        float yaw = view->subProp("Yaw")->getValue().toFloat();

        yaw -= 0.1;

        view->subProp("Yaw")->setValue(yaw);
    }
}

void MainWindow::rviz_rotateUp()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view)
    {
        float pitch = view->subProp("Pitch")->getValue().toFloat();

        pitch -= 0.1;

        view->subProp("Pitch")->setValue(pitch);
    }
}

void MainWindow::rviz_rotateDown()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view)
    {
        float pitch = view->subProp("Pitch")->getValue().toFloat();

        pitch += 0.1;

        view->subProp("Pitch")->setValue(pitch);
    }}

void MainWindow::rviz_resetView()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(!view)
        return;

    /* Reset camera distance */
    view->subProp("Distance")->setValue(10.0);

    /* Reset yaw */
    view->subProp("Yaw")->setValue(0.0);

    /* Reset pitch */
    view->subProp("Pitch")->setValue(0.785);

    /* Reset focal point */
    view->subProp("Focal Point")->setValue("0; 0; 0");

    /* Force update */
    manager_->queueRender();
}

void MainWindow::SlamToolBox()
{
    /* Already running */
    if(slam_process_->state() != QProcess::NotRunning)
    {
        ui->textEdit_log->append("SLAM already running");
        return;
    }

    QString program = "bash";
    QStringList arguments;

    arguments << "-c"
              << "source /opt/ros/humble/setup.bash && "
                 "source ~/ros2_workspace/install/setup.bash && "
                 "ros2 launch slam_toolbox online_async_launch.py";

    slam_process_->start(program, arguments);

    if(!slam_process_->waitForStarted())
    {
        ui->textEdit_log->append("Failed to start SLAM");
        return;
    }
    ui->textEdit_log->append("SLAM started");
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
