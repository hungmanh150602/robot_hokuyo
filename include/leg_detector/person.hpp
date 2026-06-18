#pragma once

#include "leg_detector/types.hpp"

struct Person
{
    Cluster cluster;

    Point2D center;

    float width;
    float height;

    float confidence;
};