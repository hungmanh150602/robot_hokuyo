#include "person_detector/visualization.hpp"

#include <rclcpp/rclcpp.hpp>

visualization_msgs::msg::MarkerArray createMarkers(const std::vector<Cluster> &clusters,
                                                   const Point2D *target,
                                                   const Point2D *camera_roi,
                                                   bool camera_detected,
                                                   float roi_radius)
{
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Clock clock;

    int id = 0;

    if (camera_roi)
    {
        visualization_msgs::msg::Marker roi;

        roi.header.frame_id = "lidar_link";
        roi.header.stamp = clock.now();
        roi.ns = "camera_roi";
        roi.id = id++;
        roi.type = visualization_msgs::msg::Marker::CYLINDER;
        roi.action = visualization_msgs::msg::Marker::ADD;

        roi.pose.position.x = camera_roi->x;
        roi.pose.position.y = camera_roi->y;
        roi.pose.position.z = 0.0;
        roi.pose.orientation.w = 1.0;

        roi.scale.x = roi_radius * 2.0;
        roi.scale.y = roi_radius * 2.0;
        roi.scale.z = 0.02;

        roi.color.r = 1.0;
        roi.color.g = 0.0;
        roi.color.b = 1.0;
        roi.color.a = 0.2;

        roi.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array.markers.push_back(roi);
    }

    // CLUSTERS
    for (const auto &cluster : clusters)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "lidar_link";
        marker.header.stamp = clock.now();
        marker.ns = "clusters";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = cluster.center.x;
        marker.pose.position.y = cluster.center.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = std::max(cluster.width, 0.05f);
        marker.scale.y = std::max(cluster.height, 0.05f);
        marker.scale.z = 0.05f;

        marker.color.r = 0.0f;
        marker.color.g = 0.4f;
        marker.color.b = 1.0f;
        marker.color.a = 0.5f;

        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array.markers.push_back(marker);
    }

    // TARGET
    if (target != nullptr)
    {
        visualization_msgs::msg::Marker target_marker;

        target_marker.header.frame_id = "lidar_link";
        target_marker.header.stamp = clock.now();
        target_marker.ns = "target";
        target_marker.id = id++;
        target_marker.type = visualization_msgs::msg::Marker::SPHERE;
        target_marker.action = visualization_msgs::msg::Marker::ADD;

        target_marker.pose.position.x = target->x;
        target_marker.pose.position.y = target->y;
        target_marker.pose.position.z = 0.15;
        target_marker.pose.orientation.w = 1.0;

        target_marker.scale.x = 0.20;
        target_marker.scale.y = 0.20;
        target_marker.scale.z = 0.20;

        // GREEN
        target_marker.color.r = 0.0f;
        target_marker.color.g = 1.0f;
        target_marker.color.b = 0.0f;
        target_marker.color.a = 1.0f;

        target_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

        marker_array.markers.push_back(target_marker);
    }

    // STATUS TEXT
    visualization_msgs::msg::Marker text;

    text.header.frame_id = "lidar_link";
    text.header.stamp = clock.now();
    text.ns = "status";
    text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;

    text.pose.position.x = 0.0;
    text.pose.position.y = 0.0;
    text.pose.position.z = 1.5;
    text.pose.orientation.w = 1.0;

    text.scale.z = 0.25;
    text.color.a = 1.0f;

    if (camera_detected)
    {
        text.text = "CAMERA DETECT";

        text.color.r = 0.0f;
        text.color.g = 1.0f;
        text.color.b = 0.0f;
    }
    else
    {
        text.text = "NO PERSON";

        text.color.r = 1.0f;
        text.color.g = 0.0f;
        text.color.b = 0.0f;
    }
    text.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_array.markers.push_back(text);

    return marker_array;
}