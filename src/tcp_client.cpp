#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <thread>
#include <cmath>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TcpClientNode : public rclcpp::Node
{
public:

    TcpClientNode()
    : Node("tcp_client_node")
    {
        /* Publisher */
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>( "/cmd_vel", 10 );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>( "/odom", 10 );

        /* TF broadcaster */
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        /* TCP */
        std::string server_ip;

        std::cout << "Enter ESP32 IP: ";
        std::getline(std::cin, server_ip);

        int server_port = 1234;

        sock_ = socket(AF_INET, SOCK_STREAM, 0);

        if (sock_ < 0)
        {
            RCLCPP_ERROR( this->get_logger(), "Cannot create socket" );
            return;
        }

        server_addr_.sin_family = AF_INET;

        server_addr_.sin_port = htons(server_port);

        inet_pton( AF_INET, server_ip.c_str(), &server_addr_.sin_addr );

        if (connect(
                sock_,
                (struct sockaddr *)&server_addr_,
                sizeof(server_addr_)) < 0)
        {
            RCLCPP_ERROR( this->get_logger(), "Connection failed" );

            close(sock_);

            return;
        }

        RCLCPP_INFO( this->get_logger(), "Connected to ESP32" );

        /* Timer update odom */
        last_time_ = this->get_clock()->now();

        timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(20),
                    std::bind(
                        &TcpClientNode::updateOdometry,
                        this)
                    );

        /* Thread đọc terminal */
        input_thread_ = std::thread(
                &TcpClientNode::inputLoop,
                this
            );
    }

    ~TcpClientNode()
    {
        close(sock_);

        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
    }

private:

    /* Thread nhập terminal */
    void inputLoop()
    {
        while (rclcpp::ok())
        {
            std::cout << "Enter message: ";

            std::string msg;

            std::getline(std::cin, msg);

            std::stringstream ss(msg);

            std::string left_str;
            std::string right_str;

            if (std::getline(ss, left_str, ',') &&
                std::getline(ss, right_str))
            {
                float left_pwm =
                    std::stof(left_str);

                float right_pwm =
                    std::stof(right_str);

                //
                // PWM -> wheel velocity
                //
                float v_l = -(0.001915 * left_pwm + 0.00053);

                float v_r = 0.001915 * right_pwm + 0.00053;

                /* Robot velocity */
                linear_velocity_ = (v_r + v_l) / 2.0;

                angular_velocity_ = (v_r - v_l) / 0.25;

                //
                // Publish velocity
                //
                geometry_msgs::msg::Twist vel_msg;

                vel_msg.linear.x =
                    linear_velocity_;

                vel_msg.angular.z =
                    angular_velocity_;

                vel_pub_->publish(vel_msg);

                //
                // Send TCP
                //
                msg += "\n";

                send(
                    sock_,
                    msg.c_str(),
                    msg.length(),
                    0
                );

                RCLCPP_INFO(
                    this->get_logger(),
                    "Sent: %s",
                    msg.c_str()
                );
            }
        }
    }

    /* Timer callback update odom */
    void updateOdometry()
    {
        auto now = this->get_clock()->now();

        double dt = (now - last_time_).seconds();

        last_time_ = now;

        /* Update pose */
        x_ += linear_velocity_ * std::cos(theta_) * dt;

        y_ += linear_velocity_ * std::sin(theta_) * dt;

        theta_ += angular_velocity_ * dt;

        /* Quaternion */
        tf2::Quaternion q;

        q.setRPY(0, 0, theta_);

        //
        // TF
        //
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = now;

        t.header.frame_id = "odom";

        t.child_frame_id =
            "base_footprint";

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);

        //
        // Odom
        //
        nav_msgs::msg::Odometry odom;

        odom.header.stamp = now;

        odom.header.frame_id = "odom";

        odom.child_frame_id =
            "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;

        odom.pose.pose.orientation.x = q.x();

        odom.pose.pose.orientation.y = q.y();

        odom.pose.pose.orientation.z = q.z();

        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = linear_velocity_;

        odom.twist.twist.angular.z = angular_velocity_;

        odom_pub_->publish(odom);

        RCLCPP_INFO(
            this->get_logger(),
            "x=%.2f y=%.2f theta=%.2f",
            x_,
            y_,
            theta_
        );
    }

private:

    /* TCP */
    int sock_;
    struct sockaddr_in server_addr_;

    /* ROS */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;

    /* Thread */
    std::thread input_thread_;

    /* Robot state */
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TcpClientNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}