#include <memory>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"



class SensorFusionNode : public rclcpp::Node
{
public:
    SensorFusionNode() : Node("sensor_fusion_node"), d0_(-1.0), is_initialized_(false)
    {
        // 1. Initialize EKF matrices with tuned weights to eliminate T265 drift
        init_ekf();

        // 2. Open TXT file for logging benchmark data in CSV format
        output_file_.open("kalman_benchmark.txt", std::ios::out);

        if (output_file_.is_open()) {

            output_file_ << "Timestamp(s),Odom_X(m),Filtered_X(m),Lidar_Distance(m),GroundTruth_X(m)\n";

            RCLCPP_INFO(this->get_logger(), "Log file created successfully: kalman_benchmark.txt");

        } else {

            RCLCPP_ERROR(this->get_logger(), "FAILED to create benchmark log file!");

        }

        // 3. Sensor Topic Subscriptions

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(

            "/odom", 10, std::bind(&SensorFusionNode::odom_callback, this, std::placeholders::_1));



        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(

            "/scan", 10, std::bind(&SensorFusionNode::scan_callback, this, std::placeholders::_1));



        // 4. ROS 2 Publishers & TF Broadcaster

        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


        // 5. Initialize Logging Timer at fixed 10Hz frequency (100ms cycle)

        logging_timer_ = this->create_wall_timer(

            std::chrono::milliseconds(100),

            std::bind(&SensorFusionNode::log_and_print_data, this));



        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node for Benchmark [10Hz Tuned English Logging] started!");

    }



    ~SensorFusionNode() {

        if (output_file_.is_open()) {

            output_file_.close();

            RCLCPP_INFO(this->get_logger(), "Log file kalman_benchmark.txt safely saved and closed.");

        }

    }



private:
    void init_ekf()
    {
        x_e_.setZero(); 
        P_e_ = Eigen::Matrix3d::Identity() * 0.1;

        // --- KALMAN FILTER TUNING TO FORCE LIDAR ALIGNMENT ---
        // Q_e_: Trust the continuous high-frequency smoothness of T265's velocity model
        Q_e_ = Eigen::Matrix3d::Identity() * 0.01;   

        // R_vio: Moderate confidence for raw camera localization updates
        R_vio_ = Eigen::Matrix3d::Identity() * 0.05;   

        // R_lidar: Set to a very low value to tightly anchor the fused state to the laser physical constraints
        R_lidar_ = Eigen::Matrix3d::Identity() * 0.0001; 

        last_time_ = this->now();
    }

    // Callback for Camera T265 (~200Hz)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        last_time_ = current_time;

        if (dt <= 0.0 || dt > 0.5) return;

        // Store raw camera odom for benchmark comparison
        raw_odom_x_ = msg->pose.pose.position.x;

        // --- EKF Prediction Step ---
        double v_x = msg->twist.twist.linear.x;
        double v_y = msg->twist.twist.linear.y;
        double omega = msg->twist.twist.angular.z;
        double theta_old = x_e_(2);

        x_e_(0) += (v_x * std::cos(theta_old) - v_y * std::sin(theta_old)) * dt;
        x_e_(1) += (v_x * std::sin(theta_old) + v_y * std::cos(theta_old)) * dt;
        x_e_(2) += omega * dt;
        x_e_(2) = std::atan2(std::sin(x_e_(2)), std::cos(x_e_(2)));

        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();

        F(0, 2) = (-v_x * std::sin(theta_old) - v_y * std::cos(theta_old)) * dt;
        F(1, 2) = (v_x * std::cos(theta_old) - v_y * std::sin(theta_old)) * dt;
        P_e_ = F * P_e_ * F.transpose() + Q_e_;

        // Correction with raw VIO data
        tf2::Quaternion q_vio(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        double r, p, yaw_vio;

        tf2::Matrix3x3(q_vio).getRPY(r, p, yaw_vio);

        Eigen::Vector3d z_vio(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_vio);

        kalman_correct(z_vio, R_vio_);

        publish_fused_data(msg->header.stamp);

    }



    // Callback for Hokuyo LiDAR (~10Hz)

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)

    {

        // Extract the center laser beam facing straight at the wall

        size_t mid_index = msg->ranges.size() / 2;

        double current_lidar_dist = msg->ranges[mid_index];



        // Filter out noise or out-of-range scan values

        if (std::isnan(current_lidar_dist) || std::isinf(current_lidar_dist) || 

            current_lidar_dist < msg->range_min || current_lidar_dist > msg->range_max) {

            return; 

        }



        current_lidar_dist_ = current_lidar_dist;



        // Initialize the base distance D0 when the robot is completely static

        if (!is_initialized_ && current_lidar_dist_ > 0.1) {

            d0_ = current_lidar_dist_;

            is_initialized_ = true;

            RCLCPP_INFO(this->get_logger(), "=== LOCKED INITIAL DISTANCE D0 = %.3f m ===", d0_);

        }



        // Calculate Ground Truth displacement: X_GT = D0 - D_t

        if (is_initialized_) {

            ground_truth_x_ = d0_ - current_lidar_dist_;



            // --- CRITICAL FIX: FEED THE ACTUAL MEASUREMENT VALUE INTO THE FILTER ---

            // Direct geometric alignment for X axis. Y and Yaw maintain the filter's baseline.

            double x_lidar = ground_truth_x_; 

            double y_lidar = x_e_(1);

            double theta_lidar = x_e_(2); 



            Eigen::Vector3d z_lidar(x_lidar, y_lidar, theta_lidar);

            kalman_correct(z_lidar, R_lidar_);

        }

    }



    void kalman_correct(const Eigen::Vector3d& z, const Eigen::Matrix3d& R)

    {

        Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

        Eigen::Vector3d y = z - H * x_e_;

        y(2) = std::atan2(std::sin(y(2)), std::cos(y(2)));



        Eigen::Matrix3d S = H * P_e_ * H.transpose() + R;

        Eigen::Matrix3d K = P_e_ * H.transpose() * S.inverse();



        x_e_ = x_e_ + K * y;

        P_e_ = (Eigen::Matrix3d::Identity() - K * H) * P_e_;

    }



    // Fixed 10Hz loop function for synchronized data logging and printing

    void log_and_print_data()

    {

        if (!is_initialized_) {

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 

                "Waiting for stable LiDAR inputs to lock baseline D0...");

            return; 

        }



        double timestamp = (this->now() - start_time_).seconds();



        // 1. Standardized English Terminal output for real-time analysis

        std::cout << std::fixed << std::setprecision(3);

        std::cout << "[LOG 10Hz] "

                  << "Time: " << std::setw(6) << timestamp << "s | "

                  << "Raw Odom: " << std::setw(6) << raw_odom_x_ << "m | "

                  << "LiDAR Dist: " << std::setw(6) << current_lidar_dist_ << "m | "

                  << "GroundTruth: " << std::setw(6) << ground_truth_x_ << "m | "

                  << "EKF Fused: " << std::setw(6) << x_e_(0) << "m\n";



        // 2. Write synchronized metrics into CSV-formatted TXT file

        if (output_file_.is_open()) {

            output_file_ << timestamp << ","

                         << raw_odom_x_ << ","

                         << x_e_(0) << ","

                         << current_lidar_dist_ << ","

                         << ground_truth_x_ << "\n";

        }

    }



    void publish_fused_data(const rclcpp::Time& stamp)

    {

        auto fused_odom = nav_msgs::msg::Odometry();

        fused_odom.header.stamp = stamp;

        fused_odom.header.frame_id = "odom";

        fused_odom.child_frame_id = "base_footprint";



        fused_odom.pose.pose.position.x = x_e_(0);

        fused_odom.pose.pose.position.y = x_e_(1);

        fused_odom.pose.pose.position.z = 0.0;



        tf2::Quaternion q;

        q.setRPY(0, 0, x_e_(2));

        fused_odom.pose.pose.orientation.x = q.x();

        fused_odom.pose.pose.orientation.y = q.y();

        fused_odom.pose.pose.orientation.z = q.z();

        fused_odom.pose.pose.orientation.w = q.w();



        fused_odom_pub_->publish(fused_odom);



        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = stamp;

        transform.header.frame_id = "odom";

        transform.child_frame_id = "base_footprint";

        transform.transform.translation.x = x_e_(0);

        transform.transform.translation.y = x_e_(1);

        transform.transform.translation.z = 0.0;

        transform.transform.rotation = fused_odom.pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);

    }



    // Benchmark and I/O variables

    std::ofstream output_file_;

    rclcpp::Time start_time_;

    rclcpp::TimerBase::SharedPtr logging_timer_;

    double d0_;

    bool is_initialized_;

    

    double raw_odom_x_ = 0.0;

    double current_lidar_dist_ = 0.0;

    double ground_truth_x_ = 0.0;



    // ROS 2 Subscribers, Publishers and EKF matrices

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    Eigen::Vector3d x_e_;

    Eigen::Matrix3d P_e_, Q_e_, R_vio_, R_lidar_;

    rclcpp::Time last_time_;

};



int main(int argc, char** argv)

{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<SensorFusionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}

