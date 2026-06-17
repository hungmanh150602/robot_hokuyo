#include "rviz_manager.h"

#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/display.hpp>

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <rviz_rendering/render_window.hpp>
#include <rviz_rendering/render_system.hpp>

RVizManager::RVizManager(QApplication *app, QWidget *widget, rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent)
{
    app_ = app;
    rviz_widget_ = widget;
    node_ = node;
}

void RVizManager::initializeRViz()
{
    /* Initialize render system */
    rviz_rendering::RenderSystem::get();
    app_->processEvents();

    /* Create render panel */
    render_panel_ = new rviz_common::RenderPanel(rviz_widget_);
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

    // add mount click
    manager_->getViewManager()->setCurrentViewControllerType(
                "rviz_default_plugins/Orbit");

    auto tool_manager = manager_->getToolManager();
    tool_manager->setCurrentTool(tool_manager->addTool(
            "rviz_default_plugins/Interact"));

    // add tool 2D Pose Estimate and 2D Goal Pose
    initial_pose_tool_ = manager_->getToolManager()->addTool(
            "rviz_default_plugins/SetInitialPose");

    goal_pose_tool_ = manager_->getToolManager()->addTool(
            "rviz_default_plugins/SetGoal");

    /* Layout */
    if(rviz_widget_->layout() == nullptr)
    {
        QVBoxLayout *layout = new QVBoxLayout(rviz_widget_);

        layout->setContentsMargins(0,0,0,0);

        layout->addWidget(render_panel_);
    }
    else
    {
        rviz_widget_->layout()->addWidget(render_panel_);
    }
    render_panel_->setFocusPolicy(Qt::StrongFocus);
    render_panel_->setMouseTracking(true);
    render_panel_->setFocus();
    render_panel_->installEventFilter(this);

    #if 1 /* Add Grid */
    auto grid = manager_->createDisplay(
                "rviz_default_plugins/Grid",
                "Grid",
                true);

    Q_UNUSED(grid);
    #endif

    #if 1 /* Fixed frame */
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    manager_->setFixedFrame("base_link");
    #endif

    #if 1 /* Add RobotModel display */
    auto robot_model = manager_->createDisplay(
                "rviz_default_plugins/RobotModel",
                "Robot Model",
                true);

    robot_model->subProp("Description Source")->setValue("Topic");
    robot_model->subProp("Description Topic")->setValue("/robot_description");

    Q_UNUSED(robot_model);
    #endif

    #if 1 /* Add TF */
    tf_display_ = manager_->createDisplay(
                "rviz_default_plugins/TF",
                "TF",
                false);
    #endif

    #if 1 /* Add LaserScan */
    laser_display_ = manager_->createDisplay(
                "rviz_default_plugins/LaserScan",
                "Laser",
                true);

    laser_display_->subProp("Style")->setValue("Points");
    laser_display_->subProp("Size (Pixels)")->setValue(3);
    laser_display_->subProp("Alpha")->setValue(1);
    laser_display_->subProp("Decay Time")->setValue(0);
    laser_display_->subProp("Color Transformer")->setValue("FlatColor");
    laser_display_->subProp("Color")->setValue(QColor(237, 51, 59));

    laser_display_->subProp("Topic")->setValue("/scan");
    #endif

    #if 1 /* map */
    map_display_ = manager_->createDisplay(
            "rviz_default_plugins/Map",
            "Map",
            true);

    map_display_->subProp("Topic")->setValue("/map");
    #endif

    #if 1 /* MarkerArray */
    marker_display_ = manager_->createDisplay(
            "rviz_default_plugins/MarkerArray",
            "Leg Markers",
            true);

    marker_display_->subProp("Topic")->setValue("/leg_markers");
    #endif

    #if 0 /* trajectory */
    auto pose =
        manager_->createDisplay(
            "rviz_default_plugins/Pose",
            "Pose",
            true);
    #endif
}

void RVizManager::zoomIn()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view)
    {
        float dist = view->subProp("Distance")->getValue().toFloat();
        dist += 1.0;
        if(dist < 0.5)
            dist = 0.5;

        view->subProp("Distance")->setValue(dist);
    }
}

void RVizManager::zoomOut()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view)
    {
        float dist = view->subProp("Distance")->getValue().toFloat();
        dist -= 1.0;

        view->subProp("Distance")->setValue(dist);
    }
}

//void RVizManager::rotateLeft()
//{
//    auto view = manager_->getViewManager()->getCurrent();

//    if(view)
//    {
//        float yaw = view->subProp("Yaw")->getValue().toFloat();
//        yaw -= 0.1;

//        view->subProp("Yaw")->setValue(yaw);
//    }
//}

void RVizManager::topView()
{
    auto view = manager_->getViewManager()->getCurrent();

    if(view) {
        /* Reset camera distance */
        view->subProp("Distance")->setValue(15.0);

        /* Reset yaw */
        view->subProp("Yaw")->setValue(0.0);

        /* Reset pitch */
        view->subProp("Pitch")->setValue(1.57);

        /* Reset focal point */
        view->subProp("Focal Point")->setValue("0; 0; 0");

        /* Force update */
        manager_->queueRender();
    }
}

QStringList RVizManager::getAllFrames()
{
    QStringList list;

    std::vector<std::string> frames = tf_buffer_->getAllFrameNames();

    for(auto &frame : frames)
    {
        list << QString::fromStdString(frame);
    }
    list.removeDuplicates();

    return list;
}

QStringList RVizManager::getLaserTopics()
{
    QStringList list;

    auto topics = node_->get_topic_names_and_types();

    for(auto &topic : topics)
    {
        QString topic_name = QString::fromStdString(topic.first);

        std::vector<std::string> types = topic.second;

        for(auto &type : types)
        {
            if(type == "sensor_msgs/msg/LaserScan")
            {
                list << topic_name;
            }
        }
    }

    list.removeDuplicates();

    return list;
}

QStringList RVizManager::getMapTopics()
{
    QStringList list;

    auto topics = node_->get_topic_names_and_types();

    for(auto &topic : topics)
    {
        QString topic_name = QString::fromStdString(topic.first);

        std::vector<std::string> types = topic.second;

        for(auto &type : types)
        {
            if(type == "nav_msgs/msg/OccupancyGrid")
            {
                list << topic_name;
            }
        }
    }

    list.removeDuplicates();

    return list;
}

QStringList RVizManager::getMarkerTopics()
{
    QStringList list;

    auto topics = node_->get_topic_names_and_types();

    for(auto &topic : topics)
    {
        std::string name = topic.first;

        for(auto &type : topic.second)
        {
            if(type == "visualization_msgs/msg/MarkerArray")
            {
                list << QString::fromStdString(name);
            }
        }
    }

    list.removeDuplicates();

    return list;
}

void RVizManager::enableTF(bool enable)
{
    if(tf_display_)
    {
        tf_display_->setEnabled(enable);
    }
}

void RVizManager::setFixedFrame(QString frame)
{
    manager_->setFixedFrame(frame);
}

void RVizManager::setLaserTopic(QString topic)
{
    if(laser_display_)
    {
        laser_display_->subProp("Topic")->setValue(topic);
    }
}

void RVizManager::setMapTopic(QString topic)
{
    if(map_display_)
    {
        map_display_->subProp("Topic")->setValue(topic);
    }
}

void RVizManager::setMarkerTopic(const QString &topic)
{
    if(marker_display_)
    {
        marker_display_->subProp("Topic")->setValue(topic);
    }
}

void RVizManager::setInitialPoseTool()
{
    auto tool_manager = manager_->getToolManager();

    tool_manager->setCurrentTool(initial_pose_tool_);
}

void RVizManager::setGoalPoseTool()
{
    auto tool_manager = manager_->getToolManager();

    tool_manager->setCurrentTool(goal_pose_tool_);
}

void RVizManager::stop()
{
}

void RVizManager::resetRViz()
{
    manager_->removeAllDisplays();

    #if 1 /* Add Grid */
    auto grid = manager_->createDisplay(
                "rviz_default_plugins/Grid",
                "Grid",
                true);

    Q_UNUSED(grid);
    #endif

    #if 1 /* Fixed frame */
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    manager_->setFixedFrame("base_link");
    #endif

    #if 1 /* Add RobotModel display */
    auto robot_model = manager_->createDisplay(
                "rviz_default_plugins/RobotModel",
                "Robot Model",
                true);

    robot_model->subProp("Description Source")->setValue("Topic");
    robot_model->subProp("Description Topic")->setValue("/robot_description");

    Q_UNUSED(robot_model);
    #endif

    #if 1 /* Add TF */
    tf_display_ = manager_->createDisplay(
                "rviz_default_plugins/TF",
                "TF",
                false);
    #endif

    #if 1 /* Add LaserScan */
    laser_display_ = manager_->createDisplay(
                "rviz_default_plugins/LaserScan",
                "Laser",
                true);

    laser_display_->subProp("Style")->setValue("Points");
    laser_display_->subProp("Size (Pixels)")->setValue(3);
    laser_display_->subProp("Alpha")->setValue(1);
    laser_display_->subProp("Decay Time")->setValue(0);
    laser_display_->subProp("Color Transformer")->setValue("FlatColor");
    laser_display_->subProp("Color")->setValue(QColor(237, 51, 59));

    laser_display_->subProp("Topic")->setValue("/scan");
    #endif

    #if 1 /* map */
    map_display_ = manager_->createDisplay(
            "rviz_default_plugins/Map",
            "Map",
            true);

    map_display_->subProp("Topic")->setValue("/map");
    #endif

    #if 0 /* trajectory */
    auto pose =
        manager_->createDisplay(
            "rviz_default_plugins/Pose",
            "Pose",
            true);
    #endif
}
