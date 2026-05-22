hello
để package vào workspace
build
source
-------- chạy node đọc dữ liệu từ lidar------------
ros2 run lidar_hokuyo publish_lidar

#--------------------- chuyển TF từ base_link sang laser ------------------------------
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser

#--------------------- chuyển TF từ base_footprint sang base_link ------------------------------
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link

#--------------------- chuyển TF từ odom sang base_footprint ------------------------------
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint

#--------------------- chạy Slam toolbox-----------------------
ros2 launch slam_toolbox online_async_launch.py

-------- chạy rviz2-------------
fix frame để odom hoặc map
LaserScan để scan
Map để map

-------- chạy node điều khiển động cơ--------------
ros2 run lidar_hokuyo tcp_client

