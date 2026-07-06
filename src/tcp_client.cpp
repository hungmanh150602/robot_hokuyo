#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>

#include <future>
#include <chrono>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

class TcpClientNode : public rclcpp::Node
{
public:
    TcpClientNode() : Node("tcp_client_node")
    {
        /* Subscriber */
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&TcpClientNode::cmdCallback, this, std::placeholders::_1));

        ip_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/ip_connect",
            10,
            std::bind(&TcpClientNode::ipCallback, this, std::placeholders::_1));

        camera_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/camera_status",
            10,
            std::bind(&TcpClientNode::cameraCallback, this, std::placeholders::_1));

        /* TCP */
        RCLCPP_INFO(
            this->get_logger(),
            "Waiting IP from GUI topic /ip_connect");
    }

    ~TcpClientNode()
    {
        close(sock_);
    }

private:
    /* TCP */
    int sock_;
    struct sockaddr_in server_addr_;

    /* ROS */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ip_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_sub_;

    double L = 0.25;
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    bool is_camera = false;

    bool connectTCP(const std::string &server_ip, int server_port)
    {
        if (sock_ > 0)
        {
            close(sock_);
            sock_ = -1;
        }

        sock_ = socket(AF_INET, SOCK_STREAM, 0);

        if (sock_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Cannot create socket: %s",
                strerror(errno));

            return false;
        }

        memset(&server_addr_, 0, sizeof(server_addr_));

        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(server_port);

        int ret = inet_pton(
            AF_INET,
            server_ip.c_str(),
            &server_addr_.sin_addr);

        if (ret <= 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid IP address");

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Try connect to %s:%d",
            server_ip.c_str(),
            server_port);

        ret = connect(
            sock_,
            (struct sockaddr *)&server_addr_,
            sizeof(server_addr_));

        if (ret < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Connection failed: %s",
                strerror(errno));

            close(sock_);
            sock_ = -1;

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Connected to ESP32");

        return true;
    }

    void ipCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string data = msg->data;
        std::string ip;
        int port = 1234;

        size_t pos = data.find(':');

        if (pos != std::string::npos)
        {
            ip = data.substr(0, pos);
            port = std::stoi(data.substr(pos + 1));
        }
        else
        {
            ip = data;
        }

        RCLCPP_INFO(this->get_logger(), "Received IP: %s", ip.c_str());

        connectTCP(ip, port);
    }

    void cameraCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_camera = msg->data;

        if (!is_camera)
        {
            std::string msg_tcp = "0,0\n";
            ssize_t bytes_sent = send(sock_, msg_tcp.c_str(), msg_tcp.length(), 0);
            RCLCPP_ERROR(this->get_logger(), "Camera Lost!!!");
            return;
        }
    }

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!is_camera)
        {
            return;
        }
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;

        // compute msg send to tcp
        double vR = linear_velocity_ + L / 2.0 * angular_velocity_;
        double vL = linear_velocity_ - L / 2.0 * angular_velocity_;

        int msg_vel_L = static_cast<int>(std::round(-(vL - 0.00053) / 0.001915));
        int msg_vel_R = static_cast<int>(std::round((vR - 0.00053) / 0.001915));

        std::string msg_tcp;

        msg_tcp = std::to_string(msg_vel_L) + "," +
                  std::to_string(msg_vel_R) + "\n";

        ssize_t bytes_sent = send(sock_, msg_tcp.c_str(), msg_tcp.length(), 0);
        if (bytes_sent < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "TCP send failed");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Sent: %s", msg_tcp.c_str());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}