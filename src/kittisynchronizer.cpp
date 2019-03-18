#include "kittireader.hpp"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

void callback(
    const sensor_msgs::ImageConstPtr& imageIn, 
    const sensor_msgs::PointCloud2ConstPtr& veloIn)
{
    ROS_INFO("Received synchronized image and pointcloud.");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(imageIn, sensor_msgs::image_encodings::BGR8);
        auto image = cv_ptr->image;
        std::cout << "shape: (" << image.rows << ", " << image.cols << ")" << std::endl;
        std::cout << "shape: (" << veloIn->height << ", " << veloIn->width << ")" << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge failure upon image receipt: %s", e.what());
    }
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

    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
