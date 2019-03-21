#include "bevprojector.hpp"
#include "continuousfusion.hpp"
#include "kittireader.hpp"

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

ContinuousFusion::ContinuousFusion(cv::Mat PRT) : 
    _xport(_nh),
    _imageSub(_nh, "/camera/image", 1),
    _veloSub(_nh, "/velodyne/points", 1),
    _sync(KittiSyncPolicy(10), _imageSub, _veloSub)
{
    _PRT = PRT;
    _publisher = _xport.advertise("/fusion/bevimage", 1);
    _sync.registerCallback(boost::bind(&ContinuousFusion::callback, this, _1, _2));
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
        cv::Mat bevImageCvFloat = BevProjector::getBevImage(image, lidar, _PRT);
        cv::Mat bevImageCv;
        bevImageCvFloat.convertTo(bevImageCv, CV_8UC3);

        cv_bridge::CvImage bevImage;
        bevImage.encoding = sensor_msgs::image_encodings::BGR8;
        bevImage.image = bevImageCv;
        _publisher.publish(bevImage.toImageMsg());
        ROS_INFO("Published BEV image.");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge failure upon image receipt: %s", e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "continuousfusion");
    std::string basedir = argv[1];
    Calibration calib = KittiReader::makeCalib(basedir);
    cv::Mat PRT = calib.getVeloToImage();
    ContinuousFusion fusionNode(PRT);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}