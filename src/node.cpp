/**
 * ROS driver interface for the 3iRobotics LiDAR.
 *
 * This package connects to the 3iRobotics LiDAR via a serial port,
 * processes the acquired data, and publishes it as a sensor_msgs::LaserScan
 * message. The node is configurable via ROS parameters.
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "C3iroboticsLidar.h"
#include "../sdk/include/CSerialConnection.h"

#include <vector>
#include <limits>
#include <cmath>

// Macro to convert degrees to radians.
#define DEG2RAD(x) ((x) * M_PI / 180.0)

// Structure representing one LiDAR data point.
struct RslidarDataComplete
{
    RslidarDataComplete()
        : signal(0),
          angle(0.0),
          distance(0.0)
    {
    }

    uint8_t signal; ///< Intensity/signal strength of the return.
    float angle;    ///< Angle (in degrees).
    float distance; ///< Distance measurement.
};

using namespace std;
using namespace everest::hwdrivers;

/**
 * @brief Publishes a ROS LaserScan message using LiDAR data.
 *
 * This function builds a sensor_msgs::LaserScan message from an array of
 * LiDAR data points and publishes it.
 *
 * @param pub         Pointer to the ROS publisher.
 * @param nodes       Pointer to an array of LiDAR data points.
 * @param node_count  Number of data points in the array.
 * @param start       Timestamp indicating the start of the scan.
 * @param scan_time   Total duration of the scan (in seconds).
 * @param angle_min   Minimum angle of the scan (in radians).
 * @param angle_max   Maximum angle of the scan (in radians).
 * @param frame_id    Reference frame for the scan data.
 */
void publish_scan(ros::Publisher *pub,
                  const RslidarDataComplete *nodes,
                  size_t node_count,
                  ros::Time start,
                  double scan_time,
                  float angle_min,
                  float angle_max,
                  const std::string &frame_id,
                  double range_min,
                  double range_max)
{
    // Create a LaserScan message.
    sensor_msgs::LaserScan scan_msg;

    // Set header fields.
    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;

    // Configure the scan angle range.
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;

    // Define the expected number of points in the scan.
    // Here, we assume one measurement per degree over 360 degrees.
    const size_t num_points = 360;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (num_points - 1);

    // Set the scan timing parameters.
    scan_msg.scan_time = scan_time;
    if (node_count > 1)
        scan_msg.time_increment = scan_time / static_cast<double>(node_count - 1);
    else
        scan_msg.time_increment = 0.0;

    // Set the range limits for valid measurements.
    // These were acquired from the product's datasheet.
    scan_msg.range_min = range_min;
    scan_msg.range_max = range_max;

    // Initialize the ranges and intensities arrays.
    scan_msg.ranges.resize(num_points, std::numeric_limits<float>::infinity());
    scan_msg.intensities.resize(num_points, 0.0);

    // Process each LiDAR data point.
    for (size_t i = 0; i < node_count; i++)
    {
        // Convert the angle (assumed to be in degrees) to an index.
        size_t current_angle = static_cast<size_t>(std::floor(nodes[i].angle));

        // Ensure that the index is within the valid range.
        if (current_angle >= num_points)
        {
            ROS_WARN("LiDAR angle out of range: %zu", current_angle);
            continue;
        }
        float read_value = nodes[i].distance;

        // Validate the reading. Out-of-range values are set to infinity.
        if (read_value < scan_msg.range_min || read_value > scan_msg.range_max)
            scan_msg.ranges[num_points - 1 - current_angle] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[num_points - 1 - current_angle] = read_value;

        // Record the intensity (signal strength).
        scan_msg.intensities[num_points - 1 - current_angle] = static_cast<float>(nodes[i].signal);
    }

    // Publish the LaserScan message.
    pub->publish(scan_msg);
}

/**
 * @brief Main entry point for the LiDAR interface node.
 *
 * This function initializes the ROS node, sets up parameters, opens a serial
 * connection to the LiDAR, and then enters a loop that retrieves LiDAR data and
 * publishes it as LaserScan messages.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return int 0 on success, or -1 on failure.
 */
int main(int argc, char *argv[])
{
    // Initialize the ROS node.
    ros::init(argc, argv, "delta_2a_lidar_node");

    // Create public and private node handles.
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Retrieve parameters from the ROS parameter server.
    std::string opt_com_path;
    nh_private.param<std::string>("serial_port", opt_com_path, "/dev/ttyUSB1");

    std::string frame_id;
    nh_private.param<std::string>("frame_id", frame_id, "laser");

    // Optionally, you could retrieve scan range parameters as well.
    double range_min, range_max;
    nh_private.param("range_min", range_min, 0.15);
    nh_private.param("range_max", range_max, 5.0);

    // Define the topic for publishing LaserScan messages.
    std::string lidar_scan_topic = "scan";
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(lidar_scan_topic, 1000);

    // Set up the serial connection parameters.
    int opt_com_baudrate = 230400;
    CSerialConnection serial_connect;
    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());

    // Attempt to open the serial port.
    if (serial_connect.openSimple())
    {
        ROS_INFO("Opened serial port %s successfully.", opt_com_path.c_str());
    }
    else
    {
        ROS_ERROR("Failed to open serial port %s.", opt_com_path.c_str());
        return -1;
    }

    ROS_INFO("3iRoboticsLidar connected.");

    // Initialize the LiDAR interface using the serial connection.
    C3iroboticsLidar robotics_lidar;
    robotics_lidar.initilize(&serial_connect);

    // Variables for tracking the scan timing.
    ros::Time start_scan_time = ros::Time::now();
    ros::Time end_scan_time;
    double scan_duration = 0.0;

    // Use ros::Rate to control the loop frequency (e.g., 100 Hz).
    ros::Rate loop_rate(100);

    // Main loop: process LiDAR data and publish LaserScan messages.
    while (ros::ok())
    {
        // Acquire data from the LiDAR.
        TLidarGrabResult result = robotics_lidar.getScanData();
        switch (result)
        {
        case LIDAR_GRAB_ING:
        {
            // Data acquisition in progress. No complete scan yet.
            break;
        }
        case LIDAR_GRAB_SUCESS:
        {
            // A complete LiDAR scan has been successfully received.
            TLidarScan lidar_scan = robotics_lidar.getLidarScan();
            size_t lidar_scan_size = lidar_scan.getSize();

            // Convert the raw LiDAR data to a vector of RslidarDataComplete.
            std::vector<RslidarDataComplete> send_lidar_scan_data;
            send_lidar_scan_data.resize(lidar_scan_size);
            for (size_t i = 0; i < lidar_scan_size; i++)
            {
                send_lidar_scan_data[i].signal = lidar_scan.signal[i];
                send_lidar_scan_data[i].angle = lidar_scan.angle[i];
                send_lidar_scan_data[i].distance = lidar_scan.distance[i];
            }

            // Define the scan angles (in radians).
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);

            // Compute the scan duration.
            // Note: ros::Time::toSec() returns seconds, so no conversion is needed.
            end_scan_time = ros::Time::now();
            scan_duration = (end_scan_time - start_scan_time).toSec();
            ROS_INFO("Received LiDAR scan with %zu data points.", lidar_scan_size);

            // Publish the scan.
            publish_scan(&scan_pub,
                         send_lidar_scan_data.data(),
                         lidar_scan_size,
                         start_scan_time,
                         scan_duration,
                         angle_min,
                         angle_max,
                         frame_id,
                         range_min,
                         range_max);

            // Reset the start time for the next scan.
            start_scan_time = end_scan_time;
            break;
        }
        case LIDAR_GRAB_ERRO:
        {
            // Log the error and consider handling it further.
            ROS_ERROR("Error while grabbing LiDAR data.");
            break;
        }
        case LIDAR_GRAB_ELSE:
        {
            ROS_WARN("Received an unexpected LiDAR grab result.");
            break;
        }
        }

        // Process any incoming ROS messages.
        ros::spinOnce();
        // Sleep to maintain the loop at the desired rate.
        loop_rate.sleep();
    }

    return 0;
}
