#include "velopublisher.hpp"
#include "kittireader.hpp"
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

VeloPublisher::VeloPublisher()
{
    _publisher = _nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/points", 1);
}

void VeloPublisher::publishCloud(cv::String fpath)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud = KittiReader::getPointcloud(fpath);
    cloud.header.frame_id = "kitti_frame";
    _publisher.publish(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kittivelo");
    VeloPublisher node = VeloPublisher();

    std::string globPattern = std::string(argv[1]) + "velodyne_points/data/*.bin";
    std::vector<cv::String> fpaths = KittiReader::globFilesHelper(globPattern);

    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        ROS_INFO("Publishing pointcloud.");
        node.publishCloud(fpaths[count]);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        if (count == fpaths.size())
        {
            count = 0;
        }
    }
    return 0;
}
