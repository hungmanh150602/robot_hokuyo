#include <memory>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "person_detector/types.hpp"
#include "person_detector/clustering.hpp"
#include "person_detector/person_filter.hpp"
#include "person_detector/visualization.hpp"

#include <fstream>
#include <chrono>
#include <iomanip>

class PersonDetector : public rclcpp::Node
{
public:
    PersonDetector() : Node("person_detector")
    {
        loadParameters();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&PersonDetector::scanCallback, this, std::placeholders::_1));

        camera_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/person_detection",
            10,
            std::bind(&PersonDetector::cameraCallback, this, std::placeholders::_1));

        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/leg_markers",
            10);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10);

        log_file_.open("person_tracking_log.csv");

        if (log_file_.is_open())
        {
            log_file_ << "time,num_persons,distance,angle\n";
        }

        RCLCPP_INFO(this->get_logger(), "Leg Detector + Person follower started!");
    }

    ~PersonDetector()
    {
        if (log_file_.is_open())
        {
            log_file_.close();
        }
    }

private:
    LegDetectorConfig config;
    std::ofstream log_file_;
    double last_log_time_ = 0.0;

    void loadParameters()
    {
        // CLUSTERING
        this->declare_parameter("cluster_eps", 0.06);

        // CONTROLLER
        this->declare_parameter("target_distance", 0.6);
        this->declare_parameter("kp_linear", 0.8);
        this->declare_parameter("kp_angular", 2.50);
        this->declare_parameter("max_linear_veclocity", 0.05);
        this->declare_parameter("max_angular_veclocity", 0.6);
        this->declare_parameter("stop_radius", 0.2);
        this->declare_parameter("danger_radius", 0.2);

        // Load value
        config.cluster_eps = this->get_parameter("cluster_eps").as_double();
        config.target_distance = this->get_parameter("target_distance").as_double();
        config.kp_linear = this->get_parameter("kp_linear").as_double();
        config.kp_angular = this->get_parameter("kp_angular").as_double();
        config.max_linear_veclocity = this->get_parameter("max_linear_veclocity").as_double();
        config.max_angular_veclocity = this->get_parameter("max_angular_veclocity").as_double();
        config.stop_radius = this->get_parameter("stop_radius").as_double();
        config.danger_radius = this->get_parameter("danger_radius").as_double();
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr camera_sub_;

    rclcpp::Time last_camera_time_;
    bool camera_connected_ = false;
    bool first_camera_msg_ = false;

    std::vector<TrackedPerson> tracks_;

    Point2D camera_pos;

    int next_track_id_ = 1;

    bool camera_detected_ = false;
    float camera_distance_ = 0.0f;
    float camera_angle_ = 0.0f;

    static constexpr int LOST_THRESHOLD = 10;   // ~1s
    static constexpr int SEARCH_THRESHOLD = 30; // ~3s

    float ROI_RADIUS = 0.0;

    // vung ban kinh xung quanh camera
    std::vector<Point2D> extractCameraROI(const std::vector<Point2D> &points)
    {
        std::vector<Point2D> roi;

        if (!camera_detected_)
            return roi;

        camera_pos.x = camera_distance_ * std::cos(camera_angle_);
        camera_pos.y = camera_distance_ * std::sin(camera_angle_);

        ROI_RADIUS = std::max(0.8f, camera_distance_ * 0.4f);

        for (const auto &p : points)
        {
            if (distance(p, camera_pos) < ROI_RADIUS)
            {
                roi.push_back(p);
            }
        }

        return roi;
    }

    //
    void updateTracks(const std::vector<Person> &persons)
    {
        for (auto &t : tracks_)
        {
            t.matched = false;
        }

        for (const auto &person : persons)
        {
            float best_dist = 999.0f;
            int best_track = -1;

            for (size_t i = 0; i < tracks_.size(); i++)
            {
                auto &t = tracks_[i];

                if (t.matched)
                    continue;

                Point2D predicted;

                predicted.x = t.center.x + t.velocity.x;

                predicted.y = t.center.y + t.velocity.y;

                float d = distance(predicted, person.center);

                if (d < best_dist)
                {
                    best_dist = d;
                    best_track = i;
                }
            }

            if (best_track >= 0 && best_dist < 0.6f)
            {
                auto &t = tracks_[best_track];

                t.velocity.x = person.center.x - t.center.x;
                t.velocity.y = person.center.y - t.center.y;
                t.center = person.center;
                t.person = person;
                t.age++;
                t.lost_frames = 0;
                t.matched = true;
            }
            else
            {
                TrackedPerson t;

                t.track_id = next_track_id_++;
                t.center = person.center;
                t.person = person;
                t.age = 1;
                t.matched = true;

                tracks_.push_back(t);
            }
        }

        for (auto &t : tracks_)
        {
            if (!t.matched)
            {
                t.lost_frames++;
            }
        }

        tracks_.erase(std::remove_if(
                          tracks_.begin(),
                          tracks_.end(),
                          [](const TrackedPerson &t)
                          {
                              return t.lost_frames > 50;
                          }),
                      tracks_.end());
    }

    // Find target next to camera
    TrackedPerson *findCameraTarget()
    {
        if (tracks_.empty())
            return nullptr;

        Point2D camera_pos;

        camera_pos.x = camera_distance_ * std::cos(camera_angle_);
        camera_pos.y = camera_distance_ * std::sin(camera_angle_);

        float best_dist = 999;
        TrackedPerson *best = nullptr;

        for (auto &t : tracks_)
        {
            float d = distance(t.center, camera_pos);

            if (d < best_dist)
            {
                best_dist = d;
                best = &t;
            }
        }
        return best;
    }

    // Follow controller
    void followTarget(const Point2D &target)
    {
        geometry_msgs::msg::Twist cmd;

        float dist = std::hypot(target.x, target.y);
        float angle = std::atan2(target.y, target.x);

        float linear = config.kp_linear * (dist - config.target_distance);
        float angular = config.kp_angular * angle;

        linear = std::clamp(linear, -config.max_linear_veclocity, config.max_linear_veclocity);
        angular = std::clamp(angular, -config.max_angular_veclocity, config.max_angular_veclocity);

        if (std::abs(dist - config.target_distance) < config.stop_radius)
        {
            linear = 0.0f;
        }

        if (dist < config.danger_radius)
        {
            linear = -0.2f;
        }

        cmd.linear.x = linear;
        cmd.angular.z = angular;

        cmd_pub_->publish(cmd);
    }

    // Camera callback
    void cameraCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        camera_distance_ = msg->x;
        camera_angle_ = -msg->y;

        camera_detected_ = (msg->z > 0.5);

        last_camera_time_ = now();
        camera_connected_ = true;
        first_camera_msg_ = true;
    }

    // Scan callback
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (!first_camera_msg_)
        {
            return;
        }

        double dt = (now() - last_camera_time_).seconds();

        if (dt > 3.0)
        {
            camera_connected_ = false;
        }

        if (!camera_connected_)
        {
            RCLCPP_ERROR(get_logger(), "Camera disconnected!");

            rclcpp::shutdown();
            return;
        }

        // Convert scan -> XY
        std::vector<Point2D> points;

        points.reserve(scan->ranges.size());

        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float r = scan->ranges[i];

            // Remove invalid
            if (!std::isfinite(r))
                continue;

            // Distance filter 0.1 - 2.5m
            if (r < 0.10 || r > 2.5)
                continue;

            float angle = scan->angle_min + i * scan->angle_increment;

            Point2D p;

            p.x = r * std::cos(angle);
            p.y = r * std::sin(angle);

            points.push_back(p);
        }

        auto roi_points = extractCameraROI(points);

        auto roi_clusters = createClusters(roi_points,
                                           config.cluster_eps,
                                           scan->angle_increment);

        auto persons = detectPersons(roi_clusters);

        updateTracks(persons);

        auto target = findCameraTarget();

        if (target)
        {
            float dist = std::hypot(target->center.x, target->center.y);
            float angle = std::atan2(target->center.y, target->center.x) * 180.0 / M_PI;

            // Timestamp
            double now_time = this->now().seconds();

            if (now_time - last_log_time_ >= 1.0)
            {
                if (log_file_.is_open())
                {
                    log_file_ << now_time << ","
                              << persons.size() << ","
                              << dist << ","
                              << angle
                              << "\n";
                }
                last_log_time_ = now_time;
            }

            RCLCPP_INFO(this->get_logger(),
                        "Tracked Person | Distance: %.3f m | Angle: %.2f deg",
                        dist, angle);

            followTarget(target->center);
        }
        else
        {
            geometry_msgs::msg::Twist cmd;

            cmd.linear.x = 0;
            cmd.angular.z = 0;

            cmd_pub_->publish(cmd);
        }

        // Publish markers
        Point2D target_marker;
        Point2D *target_ptr = nullptr;

        if (target)
        {
            target_marker = target->center;
            target_ptr = &target_marker;
        }

        auto markers = createMarkers(roi_clusters,
                                     target_ptr,
                                     camera_detected_ ? &camera_pos : nullptr,
                                     camera_detected_,
                                     ROI_RADIUS);

        marker_pub_->publish(markers);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}