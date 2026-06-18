#pragma once

#include <vector>

namespace leg_detector_config
{
    //------------------------------------------------------
    // CLUSTERING
    //------------------------------------------------------
    // Base DBSCAN eps
    constexpr float CLUSTER_EPS = 0.06f;
    // DBSCAN minimum points
    constexpr int MIN_CLUSTER_POINTS = 5;
    // Angle continuity
    constexpr int MAX_ANGLE_GAP = 20;
    // Adaptive eps multiplier
    // constexpr float ADAPTIVE_EPS_GAIN = 1.5f;
    // Extra eps offset
    constexpr float ADAPTIVE_EPS_OFFSET = 0.01f;

    //------------------------------------------------------
    // GEOMETRY FILTER
    //------------------------------------------------------
    constexpr float MIN_CLUSTER_WIDTH = 0.03f;
    constexpr float MAX_CLUSTER_WIDTH = 0.30f;

    //------------------------------------------------------
    // LEG FILTER
    //------------------------------------------------------
    // Radius
    constexpr float MIN_LEG_RADIUS = 0.01f;
    constexpr float MAX_LEG_RADIUS = 0.35f;
    // // Shape
    constexpr float MAX_ASPECT_RATIO = 2.5f;
    // // Density
    constexpr float MIN_DENSITY = 20.0f;
    // // Circularity
    constexpr float MAX_CIRCULAR_VARIANCE = 0.08f;
    // // Min lidar points
    constexpr int MIN_LEG_POINTS = 3;

    //------------------------------------------------------
    // LEG PAIR
    //------------------------------------------------------
    constexpr float MIN_LEG_DISTANCE = 0.10f;
    constexpr float MAX_LEG_DISTANCE = 0.30f;
    constexpr float MAX_RADIUS_DIFF = 0.15;
    constexpr float MAX_DENSITY_DIFF = 150.0f;
    // constexpr float MAX_LEG_HEIGHT_DIFF = 0.45f;

    //------------------------------------------------------
    // VISUALIZATION
    //------------------------------------------------------
    constexpr float LEG_MARKER_SIZE = 0.12f;
    constexpr float PERSON_MARKER_SIZE = 0.20f;
    constexpr float PAIR_LINE_WIDTH = 0.03f;
}

struct Point2D
{
    float x;
    float y;
};

struct Cluster
{
    int id = -1;

    std::vector<Point2D> points;

    Point2D center;

    float width = 0.0f;
    float height = 0.0f;

    float radius = 0.0f;

    float density = 0.0f;

    float aspect_ratio = 0.0f;

    float major_axis = 0.0f;
    float minor_axis = 0.0f;

    // float orientation = 0.0f;

    float curvature = 0.0f;

    float linearity = 0.0f;

    float arc_angle = 0.0f;
};

float distance(const Point2D &a, const Point2D &b);
Point2D computeCenter(const std::vector<Point2D> &pts);
float computeWidth(const std::vector<Point2D> &pts);
