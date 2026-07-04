#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "urg_sensor.h"
#include "urg_utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <string>

class LidarPublisher : public rclcpp::Node
{
public:
    LidarPublisher() : Node("lidar_hokuyo_node"), 
                        urg_initialized_(false), 
                        error_count_(0),
                        reconnect_attempts_(0)
    {
        // 1. Khai báo parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<std::string>("frame_id", "lidar_link");
        this->declare_parameter<double>("range_min", 0.02);
        this->declare_parameter<double>("range_max", 30.0);
        this->declare_parameter<double>("scan_time", 0.1);
        this->declare_parameter<int>("error_threshold", 5);       // số lần lỗi trước khi reconnect
        this->declare_parameter<int>("reconnect_interval_ms", 2000); // thời gian chờ giữa các lần thử

        // 2. Lấy giá trị
        port_ = this->get_parameter("port").as_string();
        baud_ = this->get_parameter("baud").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        scan_time_ = this->get_parameter("scan_time").as_double();
        error_threshold_ = this->get_parameter("error_threshold").as_int();
        reconnect_interval_ms_ = this->get_parameter("reconnect_interval_ms").as_int();

        // 3. Publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // 4. Thử kết nối lần đầu
        if (!initialize_lidar()) {
            RCLCPP_ERROR(this->get_logger(), "Initial connection failed. Will retry later.");
        }

        // 5. Timer publish (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarPublisher::publish_scan, this));
    }

    ~LidarPublisher()
    {
        if (urg_initialized_) {
            urg_close(&urg_);
            RCLCPP_INFO(this->get_logger(), "LiDAR closed on shutdown");
        }
    }

private:
    // Hàm khởi tạo (mở cổng, cấu hình, start measurement)
    bool initialize_lidar()
    {
        // Đóng kết nối cũ nếu có
        if (urg_initialized_) {
            urg_close(&urg_);
            urg_initialized_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "Opening LiDAR on %s at %d baud", port_.c_str(), baud_);
        if (urg_open(&urg_, URG_SERIAL, port_.c_str(), baud_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open LiDAR at %s", port_.c_str());
            return false;
        }

        int max_size = urg_max_data_size(&urg_);
        if (max_size <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid max_data_size");
            urg_close(&urg_);
            return false;
        }
        data_.resize(max_size);

        if (urg_start_measurement(&urg_, URG_DISTANCE, URG_SCAN_INFINITY, 0, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start measurement: %s", urg_error(&urg_));
            urg_close(&urg_);
            return false;
        }

        // Lấy min/max distance để dùng sau
        urg_distance_min_max(&urg_, &min_distance_, &max_distance_);
        urg_initialized_ = true;
        error_count_ = 0;                // reset lỗi
        RCLCPP_INFO(this->get_logger(), "LiDAR initialized successfully");
        return true;
    }

    // Hàm reconnect: thử khởi tạo lại, nếu thất bại thì lên lịch thử sau
    void attempt_reconnect()
    {
        if (reconnect_attempts_ > 0) {
            // Đã có lịch thử, không thêm mới
            return;
        }

        // Đóng kết nối cũ (nếu còn)
        if (urg_initialized_) {
            urg_close(&urg_);
            urg_initialized_ = false;
        }

        RCLCPP_WARN(this->get_logger(), "Attempting to reconnect...");
        if (initialize_lidar()) {
            RCLCPP_INFO(this->get_logger(), "Reconnection successful");
            reconnect_attempts_ = 0; // xóa lịch thử
        } else {
            RCLCPP_ERROR(this->get_logger(), "Reconnection failed. Will retry after %d ms", reconnect_interval_ms_);
            // Lên lịch thử lại sau khoảng thời gian (dùng một timer một lần)
            reconnect_attempts_ = 1; // đánh dấu đang chờ
            reconnect_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(reconnect_interval_ms_),
                std::bind(&LidarPublisher::timeout_reconnect, this));
        }
    }

    // Callback khi timer chờ reconnect hết hạn
    void timeout_reconnect()
    {
        reconnect_timer_.reset(); // hủy timer này
        reconnect_attempts_ = 0;  // cho phép thử lại
        // Gọi lại attempt_reconnect (sẽ thử lại)
        attempt_reconnect();
    }

    // Hàm publish chính
    void publish_scan()
    {
        // Nếu chưa kết nối, thử reconnect (nhưng không gọi trực tiếp nếu đang chờ)
        if (!urg_initialized_) {
            if (reconnect_attempts_ == 0) {
                attempt_reconnect();
            }
            return;
        }

        long timestamp;
        int n = urg_get_distance(&urg_, data_.data(), &timestamp);
        if (n <= 0) {
            RCLCPP_ERROR(this->get_logger(), "urg_get_distance failed: %s", urg_error(&urg_));
            error_count_++;
            if (error_count_ >= error_threshold_) {
                RCLCPP_WARN(this->get_logger(), "Too many errors (%d), triggering reconnect", error_count_);
                if (urg_initialized_) {
                    urg_close(&urg_);
                    urg_initialized_ = false;
                }
                // Thử reconnect
                if (reconnect_attempts_ == 0) {
                    attempt_reconnect();
                }
            }
            return;
        }

        // Reset lỗi khi đọc thành công
        error_count_ = 0;

        // Kiểm tra số điểm
        if (n < 2) {
            RCLCPP_WARN(this->get_logger(), "Only %d points, skipping", n);
            return;
        }

        // Tạo message
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = this->now();
        scan.header.frame_id = frame_id_;

        scan.angle_min = urg_index2rad(&urg_, 0);
        scan.angle_max = urg_index2rad(&urg_, n - 1);
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (n - 1);
        scan.scan_time = scan_time_;
        scan.time_increment = 0.0;
        scan.range_min = range_min_;
        scan.range_max = range_max_;

        scan.ranges.resize(n);
        for (int i = 0; i < n; ++i) {
            long l = data_[i];
            if ((l <= min_distance_) || (l >= max_distance_)) {
                scan.ranges[i] = std::numeric_limits<float>::infinity();
            } else {
                scan.ranges[i] = static_cast<float>(l) / 1000.0f; // mm -> m
            }
        }

        scan_pub_->publish(scan);
    }

    // Các biến thành viên
    urg_t urg_;
    bool urg_initialized_;
    int error_count_;                  // số lần đọc lỗi liên tiếp
    int error_threshold_;
    int reconnect_attempts_;           // 0: sẵn sàng, >0: đang chờ thử
    int reconnect_interval_ms_;
    std::vector<long> data_;
    long min_distance_, max_distance_;
    std::string port_;
    int baud_;
    std::string frame_id_;
    double range_min_, range_max_, scan_time_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_; // timer dùng một lần để thử lại
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}