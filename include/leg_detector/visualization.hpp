#pragma once

#include "leg_detector/types.hpp"
#include "leg_detector/person.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

visualization_msgs::msg::MarkerArray
createMarkers(
    const std::vector<Cluster>& clusters,
    const std::vector<Person>& persons);
