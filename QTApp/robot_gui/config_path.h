#ifndef CONFIG_PATH_H
#define CONFIG_PATH_H

#include <QString>

const QString user_name = "lac";
const QString work_space = "ros2_ws2";

/* Robot parameter */
#define L 0.25
#define wheel_radius 0.0325

const QString src_ros = "source /opt/ros/humble/setup.bash";

const QString src_ws = "source ~/" + work_space + "/install/setup.bash";

const QString robot_fileName = "/home/" + user_name + "/" + work_space + "/src/robot_hokuyo/urdf/Final_Assembly2.xacro";

const QString tcp_run = "ros2 run robot_hokuyo tcp_client";

const QString lidar_run = "ros2 run robot_hokuyo publish_lidar";

const QString camera_run = "ros2 run robot_hokuyo pose_camera";

const QString robot_run = "ros2 run robot_state_publisher";

const QString loadMap_run = "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/" + user_name + "/" + work_space + "/src/robot_hokuyo/maps/my_map_save.yaml";

const QString amcl_runn = "ros2 run nav2_amcl amcl --ros-args -p transform_tolerance:=2.0 -p base_frame_id:=base_footprint -p odom_frame_id:=odom -p map_frame_id:=map";

const QString nav2_runn = "ros2 launch nav2_bringup navigation_launch.py params_file:=/home/" + user_name + "/" + work_space + "/src/robot_hokuyo/nav2_params.yaml";

const QString slam_run = "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/" + user_name + "/" + work_space + "/src/robot_hokuyo/mapper_params_online_sync.yaml";

const QString camera_detect_run = "cd ~/" + work_space + "/src/robot_hokuyo/scripts && python3 final_ver_cam.py";

const QString persontracker_run = "ros2 run robot_hokuyo person_tracker_node --ros-args --params-file /home/" + user_name + "/" + work_space + "/src/robot_hokuyo/leg_detector.yaml";

#define USE_XACRO_FILE 1
#define USE_CAMERA_STATUS 1
#define USE_WHEEL_ODOM 0
#define USE_TCP_SOCKET 1
#define USE_ROS_TIMER 1
#define USE_SUB_AND_PUB 1
#define USE_RVIZ 1
#define USE_LOAD_ROBOT 1
#define USE_LIDAR 1
#define USE_CAMERA 1
#define USE_LEG_FOLLOWER 1
#define USE_SLAM_TOOLBOX 1
#define USE_NAV2 1
#define USE_BUTTON_CONTROL 1

#endif
