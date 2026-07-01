#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>
#include <cstring>
#include <cmath>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <geometry_msgs/msg/twist.hpp>

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

        /* TCP */
        std::string server_ip;

        std::cout << "Enter ESP32 IP: ";
        std::getline(std::cin, server_ip);

        int server_port = 1234;

        sock_ = socket(AF_INET, SOCK_STREAM, 0);

        if (sock_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot create socket");
            return;
        }

        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(server_port);

        inet_pton(AF_INET, server_ip.c_str(), &server_addr_.sin_addr);

        if (connect(sock_,
                    (struct sockaddr *)&server_addr_,
                    sizeof(server_addr_)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Connection failed");

            close(sock_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to ESP32");
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

    double L = 0.25;
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
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