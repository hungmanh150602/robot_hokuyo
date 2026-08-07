# robot_hokuyo
Dự án này bao gồm các tính năng xây dựng bản đồ dùng SLAM, dẫn hướng cho robot dùng NAV2, theo dõi người sử dụng lidar và camera nhận diện người.
Ngoài ra, dự án này cũng bao gồm thiết kế 1 giao diện hỗ trợ tương tác người dùng.
<img width="1356" height="793" alt="image" src="https://github.com/user-attachments/assets/376930f5-6546-43da-9b69-1e8d2f578aea" />

Đầu tiên, để có thể sử dụng được dự án này, bạn cần tạo 1 workspace ros2 sau đó thêm package này vào

# Build
colcon build --packages-select robot_hokuyo
# Cách 1: Chạy các file riêng lẻ
==============================================
# Bật giao diện Rviz
ros2 launch robot_hokuyo display.launch.py
# Chạy file kết nối tcp client để kết nối với esp32 và điều khiển động cơ
ros2 run robot_hokuyo tcp_client
# Chạy file kết nối lidar
ros2 run robot_hokuyo publish_lidar
# Chạy file kết nối camera
ros2 run robot_hokuyo pose_camera
# Chạy SLAM ToolBox để xây dựng map
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/user_name/work_space/src/robot_hokuyo/mapper_params_online_sync.yaml
# Sau đó lưu map
# Load map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/lac/ros2_ws2/my_map_save.yaml -p use_sim_time:=false
--------------- mở terminal mới ---------------
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
# Chạy AMCL để ước tính vị trí hiện tại robot
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=false -p transform_tolerance:=2.0 -p base_frame_id:=base_footprint -p odom_frame_id:=odom -p map_frame_id:=map
--------------- mở terminal mới ---------------
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
# Chạy NAV2
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/user_name/work_space/src/robot_hokuyo/nav2_params.yaml

# Tính năng đi theo người dùng
Để có thể sử dụng tính năng đi theo người dùng, trước tiên cần tắt node pose_camera vì node này sẽ xung đột kết nối camera với node nhận diện người dùng camera
Vẫn giữ node publish_camera và tcp_client
# Chạy node nhận diện người dùng camera
cd ~/work_space/src/robot_hokuyo/scripts
python3 final_ver_cam.py
# Chạy node theo dõi người
ros2 run robot_hokuyo person_tracker_node --ros-args --params-file /home/user_name/work_space/src/robot_hokuyo/leg_detector.yaml

# Cách 2: Chạy các tác vụ trên bằng cách sử dụng giao diện
# =============== run gui app =============== #
ros2 run robot_hokuyo robot_gui
