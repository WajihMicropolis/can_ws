#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core.hpp>

// Define parameters for the occupancy grid map
const double resolution = 0.1;              // meters per cell
const double min_x = -500.0, max_x = 500.0; // boundaries in X direction
const double min_y = -500.0, max_y = 500.0; // boundaries in Y direction
const double min_z = -0.21, max_z = 10.0;   // Z boundaries for clipping

ros::Publisher map_publisher;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // Convert ROS point cloud to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // Discretize the 2D plane into a grid
    int width = static_cast<int>((max_x - min_x) / resolution);
    int height = static_cast<int>((max_y - min_y) / resolution);
    cv::Mat occupancy_grid_map = cv::Mat::zeros(height, width, CV_8UC1);

    // Populate the grid cells based on point presence
    for (const auto &point : cloud->points)
    {
        // Clip the point to within the Z boundaries
        if (point.z < min_z || point.z > max_z)
            continue;

        // Calculate cell indices
        int cell_x = static_cast<int>((point.x - min_x) / resolution);
        int cell_y = static_cast<int>((point.y - min_y) / resolution); // No flipping along the y-axis

        // Check if cell indices are within map boundaries
        if (cell_x >= 0 && cell_x < width && cell_y >= 0 && cell_y < height)
        {
            // Mark the corresponding cell as occupied
            occupancy_grid_map.at<uchar>(cell_y, cell_x) = 255;
        }
    }

    // Convert the OpenCV image to ROS OccupancyGrid message
    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = ros::Time::now();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.width = width;
    occupancy_grid.info.height = height;
    occupancy_grid.info.origin.position.x = min_x;
    occupancy_grid.info.origin.position.y = min_y;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;

    occupancy_grid.data.resize(width * height);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int value = occupancy_grid_map.at<uchar>(y, x);
            occupancy_grid.data[y * width + x] = (value == 0) ? 0 : 100;
        }
    }

    // Publish the occupancy grid map
    map_publisher.publish(occupancy_grid);
}

void getRosParam(ros::NodeHandle *_nh, std::string paramName, auto &paramValue)
{
   if (!_nh->getParam(paramName, paramValue))
   {
      ROS_WARN("[rosNode] [PARAM] %s is not set", paramName.c_str());
      exit(1);
   }
   std::stringstream strg;
   strg << paramValue;
   std::string s = strg.str();
   ROS_INFO("[rosNode] [PARAM] %s = %s", paramName.c_str(), s.c_str());
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "point_cloud_to_occupancy_grid");
    ros::NodeHandle nh;
    std::string pcl_topic, map_topic;
    getRosParam(&nh, "/point_cloud_to_occupancy_grid/input_pcl_topic", pcl_topic);
    getRosParam(&nh, "/point_cloud_to_occupancy_grid/output_occupancy_grid_topic", map_topic);
    
    // Subscribe to the point cloud topic
    ros::Subscriber sub = nh.subscribe(pcl_topic, 1, pointCloudCallback);

    // Create a publisher for the occupancy grid map
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);

    // Spin
    ros::spin();

    return 0;
}
