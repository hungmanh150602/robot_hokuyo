#include "leg_detector/clustering.hpp"

#include <cmath>
#include <queue>
#include <algorithm>
#include <nanoflann.hpp>

struct PointCloudAdaptor
{
    const std::vector<Point2D> &pts;

    PointCloudAdaptor(
        const std::vector<Point2D> &points)
        : pts(points)
    {
    }

    inline size_t kdtree_get_point_count() const
    {
        return pts.size();
    }

    inline float kdtree_get_pt(
        const size_t idx,
        const size_t dim) const
    {
        return (dim == 0) ? pts[idx].x : pts[idx].y;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX &) const
    {
        return false;
    }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
    PointCloudAdaptor,
    2>;

std::vector<Cluster> createClusters(
    const std::vector<Point2D> &points,
    float base_eps,
    float angle_increment)
{
    std::vector<Cluster> clusters;

    if (points.empty())
        return clusters;

    clusters.reserve(64);

    //-----------------------------------------
    // KDTree
    //-----------------------------------------
    PointCloudAdaptor adaptor(points);

    KDTree tree(
        2,
        adaptor,
        nanoflann::KDTreeSingleIndexAdaptorParams(10));

    tree.buildIndex();

    //-----------------------------------------
    // DBSCAN params
    //-----------------------------------------
    constexpr int MIN_PTS = 5;

    //-----------------------------------------
    // labels:
    // -1 = unvisited
    //  0 = noise
    // >0 = cluster id
    //-----------------------------------------
    std::vector<int> labels(points.size(), -1);

    //-----------------------------------------
    // Neighbor cache
    //-----------------------------------------
    std::vector<std::vector<unsigned int>> neighbor_cache(points.size());

    nanoflann::SearchParams params;

    int cluster_id = 1;

    //-----------------------------------------
    // Main loop
    //-----------------------------------------
    for (size_t i = 0; i < points.size(); i++)
    {
        if (labels[i] != -1)
            continue;

        //-------------------------------------
        // Adaptive eps
        //-------------------------------------
        float r = std::hypot(points[i].x, points[i].y);

        float adaptive_eps = base_eps + r * angle_increment * 1.5f + 0.01f;

        float radius_sq = adaptive_eps * adaptive_eps;

        //-------------------------------------
        // Neighbor query
        //-------------------------------------
        auto &neighbors = neighbor_cache[i];

        if (neighbors.empty())
        {
            std::vector<std::pair<unsigned int, float>> matches;

            matches.reserve(64);

            float query_pt[2] = {points[i].x, points[i].y};

            tree.radiusSearch(
                query_pt,
                radius_sq,
                matches,
                params);

            for (const auto &m : matches)
            {
                neighbors.push_back(m.first);
            }
        }

        //-------------------------------------
        // Noise
        //-------------------------------------
        if (neighbors.size() < MIN_PTS)
        {
            labels[i] = 0;
            continue;
        }

        //-------------------------------------
        // Create cluster
        //-------------------------------------
        Cluster cluster;

        cluster.id = cluster_id++;
        cluster.points.reserve(64);

        std::queue<unsigned int> q;

        q.push(i);

        labels[i] = cluster.id;

        //-------------------------------------
        // BFS expansion
        //-------------------------------------
        while (!q.empty())
        {
            unsigned int idx = q.front();

            q.pop();

            cluster.points.push_back(points[idx]);

            //---------------------------------
            // Local adaptive eps
            //---------------------------------
            float local_r = std::hypot(points[idx].x, points[idx].y);
            float local_eps = base_eps + local_r * angle_increment * 1.5f + 0.01f;
            float local_radius_sq = local_eps * local_eps;

            //---------------------------------
            // Neighbor cache
            //---------------------------------
            auto &local_neighbors = neighbor_cache[idx];

            if (local_neighbors.empty())
            {
                std::vector<std::pair<unsigned int, float>> local_matches;

                local_matches.reserve(64);

                float local_pt[2] = {points[idx].x, points[idx].y};

                tree.radiusSearch(
                    local_pt,
                    local_radius_sq,
                    local_matches,
                    params);

                //---------------------------------
                // Angle continuity constraint
                //---------------------------------
                for (const auto &m : local_matches)
                {
                    unsigned int n = m.first;

                    if (std::abs(int(n) - int(idx)) > 25)
                        continue;

                    local_neighbors.push_back(n);
                }
            }

            //---------------------------------
            // Expand cluster
            //---------------------------------
            if (local_neighbors.size() >= MIN_PTS)
            {
                for (unsigned int n : local_neighbors)
                {
                    if (labels[n] == -1)
                    {
                        labels[n] = cluster.id;
                        q.push(n);
                    }
                }
            }
        }

        //-------------------------------------
        // Remove tiny clusters
        //-------------------------------------
        if (cluster.points.size() < MIN_PTS)
            continue;

        //-------------------------------------
        // Compute center
        //-------------------------------------
        cluster.center =
            computeCenter(
                cluster.points);

        //-------------------------------------
        // Bounding box
        //-------------------------------------
        float min_x = 1e9f;
        float max_x = -1e9f;

        float min_y = 1e9f;
        float max_y = -1e9f;

        for (const auto &p : cluster.points)
        {
            min_x = std::min(min_x, p.x);
            max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y);
            max_y = std::max(max_y, p.y);
        }

        cluster.width = max_x - min_x;
        cluster.height = max_y - min_y;
        cluster.radius = std::max(cluster.width, cluster.height) * 0.5f;

        //-------------------------------------
        // Density
        //-------------------------------------
        float area = cluster.width * cluster.height;
        cluster.density = cluster.points.size() / std::max(area, 0.0001f);

        //-------------------------------------
        // Outlier trimming
        //-------------------------------------
        std::vector<Point2D> filtered;
        filtered.reserve(cluster.points.size());

        float mean_dist = 0.0f;

        for (const auto &p : cluster.points)
        {
            mean_dist += distance(p, cluster.center);
        }

        mean_dist /= cluster.points.size();

        for (const auto &p : cluster.points)
        {
            float d = distance(p, cluster.center);

            if (d < mean_dist * 1.8f)
            {
                filtered.push_back(p);
            }
        }

        cluster.points = std::move(filtered);

        //-------------------------------------
        // Reject impossible geometry
        //-------------------------------------
        if (cluster.width < 0.03f)
            continue;

        if (cluster.width > 0.30f)
            continue;

        //-------------------------------------
        // Accept
        //-------------------------------------
        clusters.push_back(std::move(cluster));
    }
    return clusters;
}