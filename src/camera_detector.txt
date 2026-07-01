#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <chrono>

class PersonDetector : public rclcpp::Node
{
public:
    PersonDetector()
        : Node("person_detector")
    {
        // =====================================================
        // Publisher
        // =====================================================

        pub_ =
            create_publisher<
                geometry_msgs::msg::Point>(
                "/person_detection",
                10);

        // =====================================================
        // Subscribers
        // =====================================================

        left_sub_ =
            create_subscription<
                sensor_msgs::msg::Image>(
                "/fisheye_left",
                10,
                std::bind(
                    &PersonDetector::leftCallback,
                    this,
                    std::placeholders::_1));

        right_sub_ =
            create_subscription<
                sensor_msgs::msg::Image>(
                "/fisheye_right",
                10,
                std::bind(
                    &PersonDetector::rightCallback,
                    this,
                    std::placeholders::_1));

        // =====================================================
        // Load AI model
        // =====================================================

        net_ =
            cv::dnn::readNetFromCaffe(
                "MobileNetSSD_deploy.prototxt",
                "MobileNetSSD_deploy.caffemodel");

        net_.setPreferableBackend(
            cv::dnn::DNN_BACKEND_OPENCV);

        net_.setPreferableTarget(
            cv::dnn::DNN_TARGET_CPU);

        // =====================================================
        // Parameters
        // =====================================================

        SCALE_FACTOR = 0.5;

        orig_w = 848;
        orig_h = 800;

        new_w =
            static_cast<int>(
                orig_w * SCALE_FACTOR);

        new_h =
            static_cast<int>(
                orig_h * SCALE_FACTOR);

        image_size =
            cv::Size(new_w, new_h);

        // =====================================================
        // Calibration
        // =====================================================

        cx_raw =
            420.233612 * SCALE_FACTOR;

        cy_raw =
            407.466888 * SCALE_FACTOR;

        fx_raw =
            284.611786 * SCALE_FACTOR;

        fy_raw =
            285.758209 * SCALE_FACTOR;

        K1 =
            (cv::Mat_<double>(3, 3)
                 << fx_raw, 0.0, cx_raw,
             0.0, fy_raw, cy_raw,
             0.0, 0.0, 1.0);

        K2 =
            (cv::Mat_<double>(3, 3)
                 << 284.981995 * SCALE_FACTOR,
             0.0,
             428.147308 * SCALE_FACTOR,

             0.0,
             286.101807 * SCALE_FACTOR,
             405.857910 * SCALE_FACTOR,

             0.0,
             0.0,
             1.0);

        D1 =
            (cv::Mat_<double>(4, 1)
                 << -0.005314,
             0.041641,
             -0.039234,
             0.007532);

        D2 =
            (cv::Mat_<double>(4, 1)
                 << -0.005123,
             0.040124,
             -0.037531,
             0.007122);

        R_stereo =
            cv::Mat::eye(3, 3, CV_64F);

        T_stereo =
            (cv::Mat_<double>(3, 1)
                 << -0.064,
             0.0,
             0.0);

        cv::fisheye::stereoRectify(
            K1,
            D1,
            K2,
            D2,
            image_size,
            R_stereo,
            T_stereo,
            R1,
            R2,
            P1,
            P2,
            Q,
            cv::CALIB_ZERO_DISPARITY,
            image_size,
            0.0,
            1.0);

        cv::fisheye::initUndistortRectifyMap(
            K1,
            D1,
            R1,
            P1,
            image_size,
            CV_32FC1,
            map1x,
            map1y);

        cv::fisheye::initUndistortRectifyMap(
            K2,
            D2,
            R2,
            P2,
            image_size,
            CV_32FC1,
            map2x,
            map2y);

        // =====================================================
        // Stereo matcher
        // =====================================================

        NUM_DISPARITIES = 64;

        BLOCK_SIZE = 5;

        stereo_ =
            cv::StereoSGBM::create(
                0,
                NUM_DISPARITIES,
                BLOCK_SIZE);

        stereo_->setP1(8 * 3 * 5 * 5);

        stereo_->setP2(32 * 3 * 5 * 5);

        // =====================================================
        // Timer
        // =====================================================

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(333),
                std::bind(
                    &PersonDetector::process,
                    this));

        RCLCPP_INFO(
            get_logger(),
            "Person Detector Started");
    }

private:
    // =====================================================
    // Callbacks
    // =====================================================

    void leftCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        left_img_ =
            cv_bridge::toCvCopy(
                msg,
                "mono8")->image;
    }

    void rightCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        right_img_ =
            cv_bridge::toCvCopy(
                msg,
                "mono8")->image;
    }

    // =====================================================
    // Main process
    // =====================================================

    void process()
    {
        if (left_img_.empty() ||
            right_img_.empty())
        {
            return;
        }

        bool found_person = false;

        // =================================================
        // Resize
        // =================================================

        cv::Mat left_scaled;
        cv::Mat right_scaled;

        cv::resize(
            left_img_,
            left_scaled,
            image_size);

        cv::resize(
            right_img_,
            right_scaled,
            image_size);

        // =================================================
        // AI detect
        // =================================================

        cv::Mat detect_img;

        cv::resize(
            left_scaled,
            detect_img,
            cv::Size(300, 300));

        cv::Mat rgb;

        cv::cvtColor(
            detect_img,
            rgb,
            cv::COLOR_GRAY2BGR);

        cv::Mat blob =
            cv::dnn::blobFromImage(
                rgb,
                0.007843,
                cv::Size(300, 300),
                127.5);

        net_.setInput(blob);

        cv::Mat detections =
            net_.forward();

        cv::Mat detMat(
            detections.size[2],
            detections.size[3],
            CV_32F,
            detections.ptr<float>());

        // =================================================
        // Loop detections
        // =================================================

        for (int i = 0;
             i < detMat.rows;
             i++)
        {
            float confidence =
                detMat.at<float>(i, 2);

            if (confidence < 0.4)
                continue;

            int class_id =
                static_cast<int>(
                    detMat.at<float>(i, 1));

            if (class_id != 15)
                continue;

            int mx1 =
                static_cast<int>(
                    detMat.at<float>(i, 3) *
                    new_w);

            int my1 =
                static_cast<int>(
                    detMat.at<float>(i, 4) *
                    new_h);

            int mx2 =
                static_cast<int>(
                    detMat.at<float>(i, 5) *
                    new_w);

            int my2 =
                static_cast<int>(
                    detMat.at<float>(i, 6) *
                    new_h);

            // =============================================
            // Angle
            // =============================================

            double center_mx =
                (mx1 + mx2) / 2.0;

            double center_my =
                (my1 + my2) / 2.0;

            double x_ang =
                center_mx - cx_raw;

            double y_ang =
                center_my - cy_raw;

            double r =
                std::sqrt(
                    x_ang * x_ang +
                    y_ang * y_ang);

            double angle_deg;

            if (r == 0)
            {
                angle_deg = 0.0;
            }
            else
            {
                double theta =
                    r / fx_raw;

                double sin_phi =
                    x_ang / r;

                double angle_rad =
                    theta * sin_phi;

                angle_deg =
                    angle_rad *
                    180.0 /
                    CV_PI;
            }

            // =============================================
            // Rectify
            // =============================================

            cv::Mat rect_left;
            cv::Mat rect_right;

            cv::remap(
                left_scaled,
                rect_left,
                map1x,
                map1y,
                cv::INTER_LINEAR);

            cv::remap(
                right_scaled,
                rect_right,
                map2x,
                map2y,
                cv::INTER_LINEAR);

            // =============================================
            // Crop ROI
            // =============================================

            mx1 =
                std::max(0, mx1);

            my1 =
                std::max(0, my1);

            mx2 =
                std::min(new_w - 1, mx2);

            my2 =
                std::min(new_h - 1, my2);

            if (mx2 <= mx1 ||
                my2 <= my1)
            {
                continue;
            }

            cv::Rect roi(
                mx1,
                my1,
                mx2 - mx1,
                my2 - my1);

            cv::Mat crop_left =
                rect_left(roi);

            cv::Mat crop_right =
                rect_right(roi);

            // =============================================
            // Stereo disparity
            // =============================================

            cv::Mat disparity;

            stereo_->compute(
                crop_left,
                crop_right,
                disparity);

            disparity.convertTo(
                disparity,
                CV_32F,
                1.0 / 16.0);

            cv::Scalar mean_disp =
                cv::mean(disparity);

            double disp =
                mean_disp[0];

            if (disp <= 0.1)
                continue;

            // =============================================
            // Distance
            // =============================================

            double f_q =
                Q.at<double>(2, 3);

            double B =
                1.0 /
                Q.at<double>(3, 2);

            if (B < 0)
                B = -B;

            double mean_Z =
                (f_q * B) / disp;

            // =============================================
            // Publish
            // =============================================

            geometry_msgs::msg::Point msg;

            msg.x = mean_Z;

            msg.y =
                angle_deg *
                CV_PI /
                180.0;

            msg.z = 1.0;

            pub_->publish(msg);

            found_person = true;

            RCLCPP_INFO(
                get_logger(),
                "Distance: %.2f m | Angle: %.2f deg",
                mean_Z,
                angle_deg);

            break;
        }

        // =================================================
        // No person
        // =================================================

        if (!found_person)
        {
            geometry_msgs::msg::Point msg;

            msg.x = 0.0;
            msg.y = 0.0;
            msg.z = 0.0;

            pub_->publish(msg);
        }
    }

private:
    // =====================================================
    // ROS
    // =====================================================

    rclcpp::Publisher<
        geometry_msgs::msg::Point>::SharedPtr
        pub_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr
        left_sub_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image>::SharedPtr
        right_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // =====================================================
    // OpenCV
    // =====================================================

    cv::dnn::Net net_;

    cv::Mat left_img_;
    cv::Mat right_img_;

    cv::Ptr<cv::StereoSGBM> stereo_;

    // =====================================================
    // Calibration
    // =====================================================

    double SCALE_FACTOR;

    int orig_w;
    int orig_h;

    int new_w;
    int new_h;

    cv::Size image_size;

    double cx_raw;
    double cy_raw;

    double fx_raw;
    double fy_raw;

    cv::Mat K1;
    cv::Mat K2;

    cv::Mat D1;
    cv::Mat D2;

    cv::Mat R_stereo;
    cv::Mat T_stereo;

    cv::Mat R1;
    cv::Mat R2;

    cv::Mat P1;
    cv::Mat P2;

    cv::Mat Q;

    cv::Mat map1x;
    cv::Mat map1y;

    cv::Mat map2x;
    cv::Mat map2y;

    int NUM_DISPARITIES;

    int BLOCK_SIZE;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<PersonDetector>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}