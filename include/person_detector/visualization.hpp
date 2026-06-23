#pragma once

#include "person_detector/types.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

visualization_msgs::msg::MarkerArray createMarkers(const std::vector<Cluster> &clusters,
                                                   const Point2D *target,
                                                   const Point2D *camera_roi,
                                                   bool camera_detected,
                                                   float roi_radius);