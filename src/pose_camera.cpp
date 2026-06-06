#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

class PoseTfPublisher : public rclcpp::Node
{
public:
    PoseTfPublisher() : Node("pose_tf_publisher"), pipe_running_(false)
    {
        // Khởi tạo TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Khởi tạo RealSense pipeline
        std::string serial;
        if (!find_device_with_pose_stream(serial)) {
            RCLCPP_ERROR(this->get_logger(), "No RealSense device with pose stream found!");
            rclcpp::shutdown();
            return;
        }
        
        // Configure pipeline
        rs2::config cfg;
        if (!serial.empty())
            cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        
        // Start pipeline
        pipe_.start(cfg);
        pipe_running_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Pose TF publisher started successfully");
        
        // Timer để publish TF
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz
            std::bind(&PoseTfPublisher::publish_tf, this)
        );
    }
    
    ~PoseTfPublisher()
    {
        if (pipe_running_) {
            try {
                pipe_.stop();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error stopping pipeline: %s", e.what());
            }
        }
    }
    
private:
    void publish_tf()
    {
        try {
            // Get pose frames
            auto frames = pipe_.wait_for_frames();
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            if (!f) return;
            
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
            
            // Tạo TransformStamped message
            geometry_msgs::msg::TransformStamped tf_msg;
            
            // Set timestamp
            tf_msg.header.stamp = this->now();
            tf_msg.header.frame_id = "odom";  // Frame tham chiếu
            tf_msg.child_frame_id = "base_footprint";  // Frame của camera
            
            // Set translation
            tf_msg.transform.translation.x = - pose_data.translation.z;
            tf_msg.transform.translation.y = - pose_data.translation.x;
            tf_msg.transform.translation.z = pose_data.translation.y;
            
            // Set rotation (quaternion)
            tf_msg.transform.rotation.x = -pose_data.rotation.z;
            tf_msg.transform.rotation.y = -pose_data.rotation.x;
            tf_msg.transform.rotation.z =  pose_data.rotation.y;
            tf_msg.transform.rotation.w =  pose_data.rotation.w;
            // Broadcast TF
            tf_broadcaster_->sendTransform(tf_msg);
            
            // Log occasionally
            // static int counter = 0;
            // if (counter++ % 50 == 0) {
            //     RCLCPP_INFO(this->get_logger(), 
            //         "Published TF: pos(%.2f, %.2f, %.2f) - rot(%.2f, %.2f, %.2f, %.2f)",
            //         pose_data.translation.x, pose_data.translation.y, pose_data.translation.z,
            //         pose_data.rotation.w, pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z);
            // }
            
        } catch (const rs2::error & e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }
    
    bool find_device_with_pose_stream(std::string& serial)
    {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        if (devices.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "No RealSense devices found!");
            return false;
        }
        
        for (auto&& dev : devices) {
            serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            RCLCPP_INFO(this->get_logger(), "Found device: %s", serial.c_str());
            
            // Kiểm tra xem device có hỗ trợ pose stream không
            try {
                // Thử tạo pipeline với device này
                rs2::pipeline temp_pipe;
                rs2::config temp_cfg;
                temp_cfg.enable_device(serial);
                temp_cfg.enable_stream(RS2_STREAM_POSE);
                
                // Nếu không có exception, device hỗ trợ pose stream
                RCLCPP_INFO(this->get_logger(), "Device supports pose stream");
                return true;
                
            } catch (const rs2::error& e) {
                RCLCPP_WARN(this->get_logger(), "Device %s does not support pose stream: %s", 
                           serial.c_str(), e.what());
                continue;
            }
        }
        
        serial.clear();
        return false;
    }
    
    rs2::pipeline pipe_;
    bool pipe_running_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseTfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}