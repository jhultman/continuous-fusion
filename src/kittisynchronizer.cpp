#include "kittireader.hpp"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

void callback(
    const sensor_msgs::ImageConstPtr& image, 
    const sensor_msgs::PointCloud2ConstPtr& velo)
{
    ROS_INFO("Received synchronized image and pointcloud.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kittisynchronizer");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> imageSub(nh, "/camera/image", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> veloSub(nh, "/velodyne/points", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> KittiSyncPolicy;
    message_filters::Synchronizer<KittiSyncPolicy> sync(KittiSyncPolicy(10), imageSub, veloSub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate loop_rate(0.1);
    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
