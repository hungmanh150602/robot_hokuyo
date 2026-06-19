#pragma once

#include <vector>

struct LegDetectorConfig
{
    // CLUSTERING
    float cluster_eps;
    // int min_cluster_points;
    // int max_angle_gap;
    // float adaptive_eps_offset;

    // GEOMETRY
    // float min_cluster_width;
    // float max_cluster_width;

    // VISUALIZATION
    float person_marker_hight;
    float person_marker_z;

    // CONTROLLER
    float target_distance;
    float kp_linear;
    float kp_angular;
    float stop_radius;
    float danger_radius;
};

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

struct Person
{
    int id = -1;

    Point2D center;

    Cluster cluster;

    float width = 0.0f;
    float height = 0.0f;

    float confidence = 0.0f;

    // tracking
    float vx = 0.0f;
    float vy = 0.0f;

    // camera fusion
    bool matched_with_camera = false;
    bool updated = false;

    float angle = 0.0f;
    float distance = 0.0f;

    int lost_frames = 0;
};

float distance(const Point2D &a, const Point2D &b);
Point2D computeCenter(const std::vector<Point2D> &pts);
float computeWidth(const std::vector<Point2D> &pts);
