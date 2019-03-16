#ifndef KITTIREADER_HPP
#define KITTIREADER_HPP

#include "calibration.hpp"
#include <vector>
#include <opencv2/opencv.hpp>
#include "pcl_ros/point_cloud.h"

class KittiReader
{
    private:
    
    public:
        KittiReader(std::string basedir);

        static pcl::PointCloud<pcl::PointXYZI> getPointcloud(cv::String fpath);
        static std::vector<cv::Mat> getImages(std::vector<cv::String> fpaths);
        static std::vector<pcl::PointCloud<pcl::PointXYZI>> getPointclouds(std::vector<cv::String> fpaths);

        static std::map<std::string, std::vector<float>> getCalib(cv::String fpath);
        static Calibration makeCalib(cv::String basedir);

        // Reading data
        static void loadImagesAndPoints(std::string basedir);
        static std::vector<cv::String> globFilesHelper(std::string pattern);

        // Utils
        static std::vector<float> splitLineByChar(std::string line, char delim);
};

#endif