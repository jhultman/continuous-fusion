#ifndef CONTINUOUSFUSION_HPP
#define CONTINUOUSFUSION_HPP

#include <vector>
#include <opencv2/opencv.hpp>

class ContinuousFusion
{
    private:
        cv::Mat _PRT;
        Eigen::MatrixXf pcl2ToEigen(pcl::PCLPointCloud2 cloud);
        pcl::PCLPointCloud2 rosMsgToPcl2(const sensor_msgs::PointCloud2ConstPtr& cloudPtr);
        cv::Mat rosMsgToCvMat(const sensor_msgs::PointCloud2ConstPtr& cloudPtr);

    public:
        ContinuousFusion(cv::Mat PRT);
        void callback(
            const sensor_msgs::ImageConstPtr& imageIn, 
            const sensor_msgs::PointCloud2ConstPtr& veloIn);

};

#endif