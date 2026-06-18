#pragma once

#include "leg_detector/types.hpp"

#include <vector>

std::vector<Cluster> createClusters(
    const std::vector<Point2D> &points,
    float base_eps,
    float angle_increment);
