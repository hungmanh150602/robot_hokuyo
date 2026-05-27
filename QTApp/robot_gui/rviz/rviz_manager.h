#ifndef RVIZ_MANAGER_H
#define RVIZ_MANAGER_H

#include <QObject>
#include <QWidget>
#include <QApplication>
#include <QVBoxLayout>
#include <QProcess>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rviz_common
{
class RenderPanel;
class VisualizationManager;
class Display;

namespace ros_integration
{
class RosNodeAbstraction;
}
}

class RVizManager : public QObject
{
    Q_OBJECT

public:
    explicit RVizManager(QApplication *app,
                         QWidget *widget,
                         rclcpp::Node::SharedPtr node,
                         QObject *parent = nullptr);

    void initializeRViz();

    void zoomIn();
    void zoomOut();
    void rotateLeft();
    void rotateRight();
    void rotateUp();
    void rotateDown();
    void resetView();

    void enableTF(bool enable);
    void enableLaser(bool enable);
    void enableMap(bool enable);

    void setLaserTopic(QString topic);
    void setMapTopic(QString topic);
    void setFixedFrame(QString frame);

    QStringList getAllFrames();
    QStringList getLaserTopics();
    QStringList getMapTopics();
    void stop();

private:
    QApplication *app_;
    QWidget *rviz_widget_;

    rclcpp::Node::SharedPtr node_;

    rviz_common::RenderPanel *render_panel_;
    rviz_common::VisualizationManager *manager_;

    rviz_common::Display *tf_display_;
    rviz_common::Display *laser_display_;
    rviz_common::Display *map_display_;

    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_ros_node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif
