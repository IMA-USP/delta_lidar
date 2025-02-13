/*
 * Driver Interface Client
 *
 * This client subscribes to the LaserScan topic and logs scan details.
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // Check if the scan data is valid.
    if (scan->ranges.empty())
    {
        ROS_WARN("Received an empty laser scan.");
        return;
    }

    // Determine the number of data points from the ranges vector.
    size_t count = scan->ranges.size();
    ROS_INFO("Received laser scan on frame '%s' with %zu points", scan->header.frame_id.c_str(), count);
    ROS_INFO("Angle range: [%f, %f] degrees", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    // Optionally log detailed scan data at debug level.
    for (size_t i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_DEBUG("Point %zu: Angle: %f, Range: %f", i, degree, scan->ranges[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_2b_lidar_node_client");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Retrieve the topic name from the parameter server; default to "/scan"
    std::string scan_topic;
    private_nh.param<std::string>("scan_topic", scan_topic, std::string("/scan"));
    ROS_INFO("Subscribing to scan topic: %s", scan_topic.c_str());

    // Subscribe to the LaserScan topic.
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>(scan_topic, 1000, scanCallback);

    ros::spin();
    return 0;
}
