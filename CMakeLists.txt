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
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# include
include_directories(
  include/lidar_hokuyo/c
  include/lidar_hokuyo/cpp
  QTApp/robot_gui
  QTApp/robot_gui/lidar
  QTApp/robot_gui/robot
  QTApp/robot_gui/ros
  QTApp/robot_gui/rviz
  QTApp/robot_gui/slam
)

# build lidar--------------------------------
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
#-----------------------------------------

# build tcp_client------------------------
add_executable(tcp_client
  src/tcp_client.cpp
)

target_link_libraries(tcp_client
  m
)

ament_target_dependencies(tcp_client
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
#-----------------------------------------

# build fake_odom------------------------
add_executable(fake_odom
  src/fake_odom.cpp)

ament_target_dependencies(fake_odom
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)
#-----------------------------------------

# build robot_gui------------------------
add_executable(robot_gui
  QTApp/robot_gui/main.cpp
  QTApp/robot_gui/mainwindow.cpp
  QTApp/robot_gui/mainwindow.ui
  QTApp/robot_gui/lidar/lidar_manager.cpp
  QTApp/robot_gui/robot/robot_manager.cpp
  QTApp/robot_gui/ros/ros_manager.cpp
  QTApp/robot_gui/rviz/rviz_manager.cpp
  QTApp/robot_gui/slam/slam_manager.cpp
)

target_link_libraries(robot_gui
  Qt5::Core
  Qt5::Widgets
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
  sensor_msgs
  tf2
  tf2_ros

  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
)
#-----------------------------------------

install(TARGETS
  publish_lidar
  tcp_client
  fake_odom
  robot_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes launch rviz maps
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
