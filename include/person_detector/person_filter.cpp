#include "person_detector/person_filter.hpp"

#include <cmath>
#include <algorithm>

std::vector<Person> detectPersons(std::vector<Cluster> &clusters)
{
    std::vector<Person> persons;

    for (auto &cluster : clusters)
    {
        // Geometry
        float width = cluster.width;
        float height = cluster.height;

        float aspect = std::max(width, height) / std::max(0.01f, std::min(width, height));

        // Distance
        float dist = std::hypot(cluster.center.x, cluster.center.y);

        // Rules
        bool width_ok = width > 0.12f && width < 0.60f;

        bool height_ok = height > 0.08f && height < 0.60f;

        bool aspect_ok = aspect < 4.5f;

        bool density_ok = cluster.density > 5.0f;

        bool points_ok = cluster.points.size() >= 4;

        bool dist_ok = dist < 3.0f;

        // Human confidence
        float confidence = 0.0f;

        if (width_ok)
            confidence += 1.0f;

        if (height_ok)
            confidence += 1.0f;

        if (aspect_ok)
            confidence += 1.0f;

        if (density_ok)
            confidence += 1.0f;

        if (points_ok)
            confidence += 1.0f;

        if (dist_ok)
            confidence += 1.0f;

        // Accept
        if (confidence >= 5.0f)
        {
            Person p;

            p.cluster = cluster;
            p.center = cluster.center;
            p.width = width;
            p.height = height;
            p.confidence = confidence / 6.0f;

            persons.push_back(p);
        }
    }

    return persons;
}