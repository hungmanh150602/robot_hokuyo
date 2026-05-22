#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"


class FakeOdomNode : public rclcpp::Node
{
public:

    FakeOdomNode()
    : Node("fake_odom_node")
    {
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &FakeOdomNode::cmdCallback,
                this,
                std::placeholders::_1
            )
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom",
            10
        );

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(
                &FakeOdomNode::updateOdometry,
                this
            )
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Fake odom node started"
        );
    }

private:

    void cmdCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg
    )
    {
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
    }

    void updateOdometry()
    {
        double dt = 0.02;

        x_ += linear_velocity_ *
              std::cos(theta_) *
              dt;

        y_ += linear_velocity_ *
              std::sin(theta_) *
              dt;

        theta_ += angular_velocity_ * dt;

        //
        // Quaternion
        //
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        auto now = this->get_clock()->now();

        //
        // TF
        //
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";

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

        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = linear_velocity_;
        odom.twist.twist.angular.z = angular_velocity_;

        odom_pub_->publish(odom);
    }

    //
    // ROS
    //
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;

    //
    // Robot state
    //
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;

    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<FakeOdomNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}