#ifndef CONTINUOUSFUSION_HPP
#define CONTINUOUSFUSION_HPP

#include <vector>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> KittiSyncPolicy;

class ContinuousFusion
{
    private:
        cv::Mat _PRT;
        ros::NodeHandle _nh;
        image_transport::ImageTransport _xport;
        image_transport::Publisher _publisher;
        message_filters::Subscriber<sensor_msgs::Image> _imageSub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> _veloSub;
        message_filters::Synchronizer<KittiSyncPolicy> _sync;
        Eigen::MatrixXf pcl2ToEigen(pcl::PCLPointCloud2 cloud);
        pcl::PCLPointCloud2 rosMsgToPcl2(const sensor_msgs::PointCloud2ConstPtr& cloudPtr);
        cv::Mat rosMsgToCvMat(const sensor_msgs::PointCloud2ConstPtr& cloudPtr);
        void publishBevImage(cv::Mat bevImage);

    public:
        ContinuousFusion(cv::Mat PRT);
        void callback(
            const sensor_msgs::ImageConstPtr& imageIn, 
            const sensor_msgs::PointCloud2ConstPtr& veloIn);
};

#endif