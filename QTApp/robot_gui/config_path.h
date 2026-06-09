#ifndef CONFIG_PATH_H
#define CONFIG_PATH_H

#include <QString>

const QString user_name = "hungubuntu";
const QString work_space = "ros2_workspace";

const QString src_ros = "source /opt/ros/humble/setup.bash";

const QString src_ws = "source ~/" + work_space + "/install/setup.bash";

const QString robot_fileName = "/home/" + user_name + "/" + work_space + "/src/robot_hokuyo/urdf/Final_Assembly2.xacro";

const QString map_fileName = "/home/" + user_name + "/" + work_space + "/src/robot_hokuyo/maps/my_map_save.yaml";

const QString slam_fileName = "/home/" + user_name + "/" + work_space + "/src/robot_hokuyo/maps/mapper_params_online_sync.yaml";

const QString nav2_fileName = "/home/" + user_name + "/" + work_space + "/src/robot_hokuyo/NAV2/nav2_params.yaml";

const QString lidar_run = "ros2 run robot_hokuyo publish_lidar";

const QString camera_run = "ros2 run robot_hokuyo pose_camera";

const QString robot_run = "ros2 run robot_state_publisher";
#endif
