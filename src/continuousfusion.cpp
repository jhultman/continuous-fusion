#include "bevprojector.hpp"
#include "continuousfusion.hpp"
#include "kittireader.hpp"

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

ContinuousFusion::ContinuousFusion(cv::Mat PRT)
{
    _PRT = PRT;
}

pcl::PCLPointCloud2 ContinuousFusion::rosMsgToPcl2(const sensor_msgs::PointCloud2ConstPtr& cloudPtr)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloudPtr, cloud);
    pcl::PCLPointCloud2 pcl2;
    pcl::toPCLPointCloud2(cloud, pcl2);
    return pcl2;
}

Eigen::MatrixXf ContinuousFusion::pcl2ToEigen(pcl::PCLPointCloud2 cloud)
{
    Eigen::MatrixXf eigenCloud;
    pcl::getPointCloudAsEigen(cloud, eigenCloud);
    return eigenCloud;
}

cv::Mat ContinuousFusion::rosMsgToCvMat(const sensor_msgs::PointCloud2ConstPtr& cloudPtr)
{
    // Avoid memcpy through underlying eigen mat (I think...)
    auto pcl2 = rosMsgToPcl2(cloudPtr);
    auto eigenCloud = pcl2ToEigen(pcl2);
    cv::Mat cvCloud;
    eigen2cv(eigenCloud, cvCloud);
    return cvCloud;
}

void ContinuousFusion::callback(
    const sensor_msgs::ImageConstPtr& imageIn, 
    const sensor_msgs::PointCloud2ConstPtr& veloIn)
{
    ROS_INFO("Received synchronized image and pointcloud.");
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imageIn, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        cv::Mat lidar = rosMsgToCvMat(veloIn);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge failure upon image receipt: %s", e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kittisynchronizer");
    ros::NodeHandle nh;

    ContinuousFusion fusionNode = ContinuousFusion(cv::Mat::zeros(3, 4, CV_32F));
    message_filters::Subscriber<sensor_msgs::Image> imageSub(nh, "/camera/image", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> veloSub(nh, "/velodyne/points", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> KittiSyncPolicy;
    message_filters::Synchronizer<KittiSyncPolicy> sync(KittiSyncPolicy(10), imageSub, veloSub);
    sync.registerCallback(boost::bind(&ContinuousFusion::callback, &fusionNode, _1, _2));

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