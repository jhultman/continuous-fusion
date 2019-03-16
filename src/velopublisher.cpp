#include "kittireader.hpp"

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

void callback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input)
{
    ROS_INFO("Received pointcloud.");
    std::cout << "shape: (" << input->width << ", " << input->height << ")" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kittivelo");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/points", 1);
    ros::Subscriber subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/points", 1, callback);

    std::string globPattern = std::string(argv[1]) + "velodyne_points/data/*.bin";
    std::vector<cv::String> fpaths = KittiReader::globFilesHelper(globPattern);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> pointclouds = KittiReader::getPointclouds(fpaths);

    auto cloud = pointclouds[0];
    std::cout << "shape: (" << cloud.width << ", " << cloud.height << ")" << std::endl;

    ros::Rate loop_rate(0.05);
    int count = 0;
    while (ros::ok())
    {
        ROS_INFO("Publishing pointcloud.");
        publisher.publish(cloud);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
