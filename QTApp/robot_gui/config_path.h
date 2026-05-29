#ifndef CONFIG_PATH_H
#define CONFIG_PATH_H

#include <QString>

const QString src_ros = "source /opt/ros/humble/setup.bash";

const QString src_ws = "source ~/ros2_workspace/install/setup.bash";

const QString robot_fileName = "/home/hungubuntu/ros2_workspace/src/robot_hokuyo/urdf/Final_Assembly2.xacro";

const QString lidar_run = "ros2 run robot_hokuyo publish_lidar";

const QString robot_run = "ros2 run robot_state_publisher";
#endif
