#include "leg_detector/visualization.hpp"

#include <rclcpp/rclcpp.hpp>

visualization_msgs::msg::MarkerArray
createMarkers(
    const std::vector<Cluster>& clusters,
    const std::vector<Person>& persons)
{
    visualization_msgs::msg::MarkerArray marker_array;

    rclcpp::Clock clock;

    int id = 0;

    //----------------------------------------------------
    // Draw clusters
    //----------------------------------------------------
    for (const auto& cluster : clusters)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "lidar_link";
        marker.header.stamp = clock.now();
        marker.ns = "clusters";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        //----------------------------------------
        // Position
        //----------------------------------------
        marker.pose.position.x = cluster.center.x;
        marker.pose.position.y = cluster.center.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //----------------------------------------
        // Cluster size
        //----------------------------------------
        marker.scale.x = std::max(cluster.width, 0.05f);
        marker.scale.y = std::max(cluster.height, 0.05f);
        marker.scale.z = 0.05f;

        //----------------------------------------
        // BLUE
        //----------------------------------------
        marker.color.r = 0.0f;
        marker.color.g = 0.4f;
        marker.color.b = 1.0f;
        marker.color.a = 0.5f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_array.markers.push_back(marker);
    }

    //----------------------------------------------------
    // Draw detected persons
    //----------------------------------------------------
    for (const auto& person : persons)
    {
        //----------------------------------------
        // PERSON BODY
        //----------------------------------------
        visualization_msgs::msg::Marker body;

        body.header.frame_id = "lidar_link";
        body.header.stamp = clock.now();
        body.ns = "persons";
        body.id = id++;
        body.type = visualization_msgs::msg::Marker::CYLINDER;
        body.action =visualization_msgs::msg::Marker::ADD;

        //----------------------------------------
        // Position
        //----------------------------------------
        body.pose.position.x = person.center.x;
        body.pose.position.y = person.center.y;
        body.pose.position.z = 0.0;
        body.pose.orientation.w = 1.0;

        //----------------------------------------
        // Body size
        //----------------------------------------
        body.scale.x = std::max(person.width, 0.15f);
        body.scale.y = std::max(person.width, 0.15f);
        body.scale.z = 0.8f;

        //----------------------------------------
        // YELLOW
        //----------------------------------------
        body.color.r = 1.0f;
        body.color.g = 1.0f;
        body.color.b = 0.0f;
        body.color.a = 0.8f;
        body.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_array.markers.push_back(body);

        //----------------------------------------
        // PERSON CENTER
        //----------------------------------------
        visualization_msgs::msg::Marker center;

        center.header.frame_id = "lidar_link";
        center.header.stamp = clock.now();
        center.ns = "person_center";
        center.id = id++;
        center.type = visualization_msgs::msg::Marker::SPHERE;
        center.action = visualization_msgs::msg::Marker::ADD;

        center.pose.position.x = person.center.x;
        center.pose.position.y = person.center.y;
        center.pose.position.z = 0.05;
        center.pose.orientation.w = 1.0;

        //----------------------------------------
        // Size
        //----------------------------------------
        center.scale.x = 0.08f;
        center.scale.y = 0.08f;
        center.scale.z = 0.08f;

        //----------------------------------------
        // RED
        //----------------------------------------
        center.color.r = 1.0f;
        center.color.g = 0.0f;
        center.color.b = 0.0f;
        center.color.a = 1.0f;
        center.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_array.markers.push_back(center);

        //----------------------------------------
        // TEXT
        //----------------------------------------
        visualization_msgs::msg::Marker text;

        text.header.frame_id = "lidar_link";
        text.header.stamp = clock.now();
        text.ns = "person_text";
        text.id = id++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;

        text.pose.position.x = person.center.x;
        text.pose.position.y = person.center.y;
        text.pose.position.z = 1.0;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.15f;

        //----------------------------------------
        // WHITE
        //----------------------------------------
        text.color.r = 1.0f;
        text.color.g = 1.0f;
        text.color.b = 1.0f;
        text.color.a = 1.0f;
        text.text = "Human";
        text.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_array.markers.push_back(text);
    }

    return marker_array;
}