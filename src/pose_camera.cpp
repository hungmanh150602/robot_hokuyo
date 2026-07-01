#include <librealsense2/rs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <iostream>
#include <iomanip>
#include <cmath>

class PoseOdomPublisher : public rclcpp::Node
{
public:
    PoseOdomPublisher() : Node("pose_odom_publisher"), pipe_running_(false)
    {
        // Publisher /odom
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/camera_status", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // map -> odom
        geometry_msgs::msg::TransformStamped static_tf;

        static_tf.header.stamp = this->now();
        static_tf.header.frame_id = "map";
        static_tf.child_frame_id = "odom";

        static_tf.transform.translation.x = 0.0;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.0;

        static_tf.transform.rotation.x = 0.0;
        static_tf.transform.rotation.y = 0.0;
        static_tf.transform.rotation.z = 0.0;
        static_tf.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(static_tf);

        // tìm camera
        std::string serial;
        if (!find_device_with_pose_stream(serial))
        {
            RCLCPP_ERROR(this->get_logger(), "No RealSense device with pose stream found!");
            rclcpp::shutdown();
            return;
        }

        // Configure pipeline
        rs2::config cfg;
        if (!serial.empty())
            cfg.enable_device(serial);

        cfg.disable_all_streams();
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

        // Start pipeline
        pipe_.start(cfg);
        pipe_running_ = true;

        RCLCPP_INFO(this->get_logger(), "Pose Odom publisher started successfully");

        // Timer publish /odom
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PoseOdomPublisher::publish_odom, this));

        last_frame_time_ = this->now();
    }

    ~PoseOdomPublisher()
    {
        if (pipe_running_)
        {
            try
            {
                pipe_.stop();
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error stopping pipeline: %s", e.what());
            }
        }
    }

private:
    void publish_camera_status(bool status)
    {
        std_msgs::msg::Bool msg;
        msg.data = status;

        status_pub_->publish(msg);
    }

    void publish_odom()
    {
        try
        {
            // Get pose frames
            rs2::frameset frames;

            if (!pipe_.poll_for_frames(&frames))
            {
                auto now = this->now();

                // Chưa từng nhận frame nào
                if (!first_frame_received_)
                {
                    return;
                }

                // Đã từng nhận frame nhưng bị timeout
                double dt = (now - last_frame_time_).seconds();
                if (dt > 1.0)
                {
                    publish_camera_status(false);

                    RCLCPP_ERROR(this->get_logger(), "Camera frame timeout -> shutdown");

                    if (pipe_running_)
                    {
                        try
                        {
                            pipe_.stop();
                            pipe_running_ = false;
                        }
                        catch (...)
                        {
                        }
                    }
                    rclcpp::shutdown();
                }
                return;
            }

            publish_camera_status(true);

            first_frame_received_ = true;
            last_frame_time_ = this->now();

            auto f = frames.first_or_default(RS2_STREAM_POSE);

            if (!f)
                return;

            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

            // Publish /odom
            nav_msgs::msg::Odometry odom_msg;

            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_footprint";

            odom_msg.pose.pose.position.x = -pose_data.translation.z;
            odom_msg.pose.pose.position.y = -pose_data.translation.x;
            odom_msg.pose.pose.position.z = pose_data.translation.y;

            odom_msg.pose.pose.orientation.x = -pose_data.rotation.z;
            odom_msg.pose.pose.orientation.y = -pose_data.rotation.x;
            odom_msg.pose.pose.orientation.z = pose_data.rotation.y;
            odom_msg.pose.pose.orientation.w = pose_data.rotation.w;

            odom_msg.twist.twist.linear.x = -pose_data.velocity.z;
            odom_msg.twist.twist.linear.y = -pose_data.velocity.x;
            odom_msg.twist.twist.linear.z = pose_data.velocity.y;

            odom_msg.twist.twist.angular.x = -pose_data.angular_velocity.z;
            odom_msg.twist.twist.angular.y = -pose_data.angular_velocity.x;
            odom_msg.twist.twist.angular.z = pose_data.angular_velocity.y;

            odom_pub_->publish(odom_msg);

            // Publish /tf
            geometry_msgs::msg::TransformStamped tf_msg;

            tf_msg.header.stamp = odom_msg.header.stamp;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = "base_footprint";

            tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
            tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
            tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

            tf_broadcaster_->sendTransform(tf_msg);
        }
        catch (const rs2::error &e)
        {
            publish_camera_status(false);

            RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());

            if (pipe_running_)
            {
                try
                {
                    pipe_.stop();
                    pipe_running_ = false;
                }
                catch (...)
                {
                }
            }

            RCLCPP_ERROR(this->get_logger(), "Camera disconnected -> shutting down node");

            rclcpp::shutdown();
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    bool find_device_with_pose_stream(std::string &serial)
    {
        rs2::context ctx;
        auto devices = ctx.query_devices();

        if (devices.size() == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "No RealSense devices found!");
            return false;
        }

        for (auto &&dev : devices)
        {
            serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

            RCLCPP_INFO(this->get_logger(), "Found device: %s", serial.c_str());
            // Kiểm tra pose stream
            try
            {
                rs2::pipeline temp_pipe;
                rs2::config temp_cfg;
                temp_cfg.enable_device(serial);
                temp_cfg.enable_stream(RS2_STREAM_POSE);
                RCLCPP_INFO(this->get_logger(), "Device supports pose stream");

                return true;
            }
            catch (const rs2::error &e)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Device %s does not support pose stream: %s",
                            serial.c_str(),
                            e.what());
                continue;
            }
        }
        serial.clear();
        return false;
    }

    rs2::pipeline pipe_;
    bool pipe_running_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_frame_time_;
    bool first_frame_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseOdomPublisher>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
