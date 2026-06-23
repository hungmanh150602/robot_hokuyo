#include "person_detector/types.hpp"

#include <cmath>

float distance(const Point2D &a, const Point2D &b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

Point2D computeCenter(const std::vector<Point2D> &pts)
{
    Point2D c{0.0f, 0.0f};

    for (const auto &p : pts)
    {
        c.x += p.x;
        c.y += p.y;
    }

    c.x /= pts.size();
    c.y /= pts.size();

    return c;
}

float computeWidth(
    const std::vector<Point2D> &pts)
{
    float max_dist = 0.0f;

    for (size_t i = 0; i < pts.size(); i++)
    {
        for (size_t j = i + 1; j < pts.size(); j++)
        {
            float d = distance(pts[i], pts[j]);

            if (d > max_dist)
                max_dist = d;
        }
    }

    return max_dist;
}