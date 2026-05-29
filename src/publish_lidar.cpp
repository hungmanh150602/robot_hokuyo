/*!
  \~japanese
  \example get_distance.c �����f�[�^���擾����
  \~english
  \example get_distance.c Obtains distance data
  \~
  \author Satofumi KAMIMURA

  $Id$
*/
#include <rclcpp/rclcpp.hpp>

#include "urg_sensor.h"
#include "urg_utils.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <sensor_msgs/msg/laser_scan.hpp>

static void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
    printf("=== SCAN (timestamp: %ld) ===\n", time_stamp);
#if 0
    int front_index;

    (void)data_n;

    // \~japanese �O���̃f�[�^�݂̂�\��
    // \~english Shows only the front step
    front_index = urg_step2index(urg, 0);
    printf("%ld [mm], (%ld [msec])\n", data[front_index], time_stamp);

#else
    (void)time_stamp;

    int i;
    long min_distance;
    long max_distance;

    // \~japanese �S�Ẵf�[�^�� X-Y �̈ʒu��\��
    // \~english Prints the X-Y coordinates for all the measurement points
    urg_distance_min_max(urg, &min_distance, &max_distance);
    for (i = 0; i < data_n; ++i)
    {
        long l = data[i];

        if ((l <= min_distance) || (l >= max_distance))
        {
            continue;
        }

#if 0
            double radian;
            long x;
            long y;

            radian = urg_index2rad(urg, i);
            x = (long)(l * cos(radian));
            y = (long)(l * sin(radian));
            printf("(%ld, %ld), ", x, y);
#else
        printf("%ld ", l);
#endif
    }
    printf("\n=============================\n\n");
#endif
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("lidar_hokuyo_node");
    auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // enum
    // {
    //     CAPTURE_TIMES = 1,
    // };
    urg_t urg;
    long *data = NULL;
    long time_stamp;
    int n;
    // int i;

    // connection
    if (urg_open(&urg, URG_SERIAL, "/dev/ttyACM0", 115200) < 0)
    {
        printf("Cannot open LiDAR\n");
        return 1;
    }

    data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
    if (!data)
    {
        perror("urg_max_index()");
        return 1;
    }

    // \~japanese �f�[�^�擾
    // \~english Gets measurement data
#if 0
    // \~japanese �f�[�^�̎擾�͈͂�ύX����ꍇ
    // \~english Case where the measurement range (start/end steps) is defined
    urg_set_scanning_parameter(&urg,
                               urg_deg2step(&urg, -90),
                               urg_deg2step(&urg, +90), 0);
#endif

    urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0, 1);
    RCLCPP_INFO(node->get_logger(), "LiDAR started");
    while (rclcpp::ok())
    {
        n = urg_get_distance(&urg, data, &time_stamp);
        if (n <= 0)
        {
            RCLCPP_ERROR(node->get_logger(), "urg_get_distance: %s", urg_error(&urg));
            printf("urg_get_distance: %s\n", urg_error(&urg));
            free(data);
            urg_close(&urg);
            return 1;
        }
        // print_data(&urg, data, n, time_stamp);

        // Publish topic /scan
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = node->now();
        scan.header.frame_id = "lidar_link";

        scan.angle_min = urg_index2rad(&urg, 0);
        scan.angle_max = urg_index2rad(&urg, n - 1);

        scan.angle_increment = (scan.angle_max - scan.angle_min) / (n - 1);
        scan.scan_time = 0.1;
        scan.time_increment = 0.0;

        scan.range_min = 0.02;
        scan.range_max = 30.0;
        scan.ranges.resize(n);

        long min_distance;
        long max_distance;

        urg_distance_min_max( &urg, &min_distance, &max_distance);

        for (int i = 0; i < n; ++i)
        {
            long l = data[i];

            if ((l <= min_distance) || (l >= max_distance))
            {
                scan.ranges[i] =
                    std::numeric_limits<float>::infinity();
            }
            else
            {
                scan.ranges[i] =
                    l / 1000.0;
            }
        }

        publisher->publish(scan);
        rclcpp::spin_some(node);
    }

    // \~japanese �ؒf
    // \~english Disconnects
    free(data);
    urg_close(&urg);

    RCLCPP_INFO(node->get_logger(), "LiDAR node stopped");

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
