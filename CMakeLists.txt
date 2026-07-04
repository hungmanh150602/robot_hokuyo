cmake_minimum_required(VERSION 3.8)
project(robot_hokuyo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui Network)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find RealSense2
find_library(REALSENSE2_LIB realsense2)
find_path(REALSENSE2_INCLUDE librealsense2/rs.hpp)

if(NOT REALSENSE2_LIB OR NOT REALSENSE2_INCLUDE)
   message(FATAL_ERROR "RealSense2 not found. Please install librealsense2-dev")
endif()

# include directories
include_directories(
  include
  include/lidar_hokuyo/c
  QTApp/robot_gui
  QTApp/robot_gui/tcp_client
  QTApp/robot_gui/camera
  QTApp/robot_gui/lidar
  QTApp/robot_gui/person_tracker
  QTApp/robot_gui/robot
  QTApp/robot_gui/rviz
  QTApp/robot_gui/slam
  QTApp/robot_gui/nav2
)

# ===================== build publish_lidar =====================
add_executable(publish_lidar
  src/publish_lidar.cpp

  include/lidar_hokuyo/c/urg_sensor.c
  include/lidar_hokuyo/c/urg_utils.c
  include/lidar_hokuyo/c/urg_connection.c
  include/lidar_hokuyo/c/urg_serial.c
  include/lidar_hokuyo/c/urg_tcpclient.c
  include/lidar_hokuyo/c/urg_ring_buffer.c
)

target_link_libraries(publish_lidar
  m
)

ament_target_dependencies(publish_lidar
  rclcpp
  sensor_msgs
)
#--------------------------------------------------------------------

# ===================== build tcp_client =====================
add_executable(tcp_client
  src/tcp_client.cpp
)

ament_target_dependencies(tcp_client
  rclcpp
  geometry_msgs
)
#--------------------------------------------------------------------

# ===================== build fake_odom =====================
add_executable(fake_odom
  src/fake_odom.cpp)

ament_target_dependencies(fake_odom
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
#--------------------------------------------------------------------

# ===================== build pose_camera =====================
add_executable(pose_camera
 src/pose_camera.cpp)

target_include_directories(pose_camera PRIVATE
   ${REALSENSE2_INCLUDE}
)

target_link_libraries(pose_camera
   ${REALSENSE2_LIB}
   ${OpenCV_LIBS}
)

target_compile_features(pose_camera PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(pose_camera
 rclcpp
 nav_msgs
 sensor_msgs
 tf2
 tf2_ros
)
#--------------------------------------------------------------------

# ===================== build leg_follower_node =====================
add_executable(leg_follower_node
  src/leg_follower_node.cpp

  include/leg_detector/clustering.cpp
  include/leg_detector/person_filter.cpp
  include/leg_detector/visualization.cpp
  include/leg_detector/types.cpp
)

ament_target_dependencies(leg_follower_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)
#--------------------------------------------------------------------

# ===================== build person_tracker_node =====================
add_executable(person_tracker_node
  src/person_tracker_node.cpp

  include/person_detector/clustering.cpp
  include/person_detector/person_filter.cpp
  include/person_detector/visualization.cpp
  include/person_detector/types.cpp
)

ament_target_dependencies(person_tracker_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)
#--------------------------------------------------------------------

# ===================== build robot_gui =====================
add_executable(robot_gui
  QTApp/robot_gui/main.cpp
  QTApp/robot_gui/mainwindow.cpp
  QTApp/robot_gui/mainwindow.ui
  QTApp/robot_gui/tcp_client/tcp_manager.cpp
  QTApp/robot_gui/camera/camera_manager.cpp
  QTApp/robot_gui/lidar/lidar_manager.cpp
  QTApp/robot_gui/person_tracker/person_manager.cpp
  QTApp/robot_gui/robot/robot_manager.cpp
  QTApp/robot_gui/rviz/rviz_manager.cpp
  QTApp/robot_gui/slam/slam_manager.cpp
  QTApp/robot_gui/nav2/NAV2_manager.cpp
)

target_link_libraries(robot_gui
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::Network

  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

ament_target_dependencies(robot_gui
  rclcpp
  geometry_msgs
  nav_msgs

  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
)
#--------------------------------------------------------------------

install(TARGETS
  pose_camera
  publish_lidar
  tcp_client
  fake_odom
  leg_follower_node
  person_tracker_node
  robot_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
# Install Python executables
install(
  PROGRAMS
    scripts/load_map.py
    scripts/a_star.py
    scripts/path_follower.py
    scripts/AStar_Navigation.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  meshes
  launch
  rviz
  maps
  scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
