#include <memory>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "leg_detector/types.hpp"
#include "leg_detector/clustering.hpp"
#include "leg_detector/person_filter.hpp"
#include "leg_detector/visualization.hpp"

class LegDetector : public rclcpp::Node
{
public:
    LegDetector() : Node("leg_detector")
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&LegDetector::scanCallback, this, std::placeholders::_1));

        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/leg_markers",
            10);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10);

        RCLCPP_INFO(this->get_logger(), "Leg Detector + Person follower started!");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    //----------------------------------------------
    // Follow controller
    //----------------------------------------------
    void followPerson(const std::vector<Person> &persons)
    {
        geometry_msgs::msg::Twist cmd;

        //------------------------------------------
        // No target
        //------------------------------------------
        if (persons.empty())
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;

            cmd_pub_->publish(cmd);
            return;
        }

        //------------------------------------------
        // Find nearest person
        //------------------------------------------
        int best_idx = -1;
        float best_dist = 999.0f;

        for (size_t i = 0; i < persons.size(); i++)
        {
            float d = std::hypot(persons[i].center.x, persons[i].center.y);

            if (d < best_dist)
            {
                best_dist = d;
                best_idx = i;
            }
        }

        if (best_idx < 0)
            return;

        //------------------------------------------
        // Target
        //------------------------------------------
        float x = persons[best_idx].center.x;
        float y = persons[best_idx].center.y;

        //------------------------------------------
        // Distance + angle
        //------------------------------------------
        float distance = best_dist;
        float angle = std::atan2(y, x);

        //------------------------------------------
        // Controller
        //------------------------------------------
        float target_distance = 1.5f;
        float linear = 1.2f * (distance - target_distance);
        float angular = 2.5f * angle;

        //------------------------------------------
        // Dead zone
        //------------------------------------------
        if (std::abs(distance - target_distance) < 0.15f)
        {
            linear = 0.0f;
            angular = 0.0f;
        }

        //------------------------------------------
        // Safety reverse
        //------------------------------------------
        if (distance < 0.5f)
        {
            linear = -0.5f;
        }

        //------------------------------------------
        // Limit
        //------------------------------------------
        linear = std::clamp(linear, -0.5f, 0.5f);
        angular = std::clamp(angular, -1.0f, 1.0f);

        //------------------------------------------
        // Publish
        //------------------------------------------
        cmd.linear.x = linear;
        cmd.angular.z = angular;

        cmd_pub_->publish(cmd);
    }

    //----------------------------------------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Convert scan -> XY
        std::vector<Point2D> points;

        points.reserve(scan->ranges.size());

        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float r = scan->ranges[i];

            // Remove invalid
            if (!std::isfinite(r))
                continue;

            // Distance filter 0.1 - 2.5m
            if (r < 0.10 || r > 2.5)
                continue;

            float angle = scan->angle_min + i * scan->angle_increment;

            Point2D p;

            p.x = r * std::cos(angle);
            p.y = r * std::sin(angle);

            points.push_back(p);
        }

        // Clustering
        auto clusters = createClusters(points, leg_detector_config::CLUSTER_EPS, scan->angle_increment);
        //------------------------------------------------------
        // Detect persons
        //------------------------------------------------------
        auto persons = detectPersons(clusters);
        //------------------------------------------
        // Follow
        //------------------------------------------
        followPerson(persons);
        //------------------------------------------------------
        // Publish markers
        //------------------------------------------------------
        auto markers = createMarkers(clusters, persons);

        marker_pub_->publish(markers);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LegDetector>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
