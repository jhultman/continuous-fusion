#ifndef VELOPUBLISHER_HPP
#define VELOPUBLISHER_HPP

#include <opencv2/opencv.hpp>
#include "kittireader.hpp"
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class VeloPublisher
{
    private:
        ros::NodeHandle _nh;
        ros::Publisher _publisher;

    public:
        VeloPublisher();
        void publishCloud(cv::String fpath);
};

#endif