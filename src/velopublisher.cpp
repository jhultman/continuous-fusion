#include "kittireader.hpp"
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kittivelo");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/points", 1);

    std::string globPattern = std::string(argv[1]) + "velodyne_points/data/*.bin";
    std::vector<cv::String> fpaths = KittiReader::globFilesHelper(globPattern);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> pointclouds = KittiReader::getPointclouds(fpaths);
    auto cloud = pointclouds[0];
    cloud.header.frame_id = "kitti_frame";

    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok())
    {
        ROS_INFO("Publishing pointcloud.");
        std::cout << "Header: " << cloud.header << std::endl;
        publisher.publish(cloud);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
