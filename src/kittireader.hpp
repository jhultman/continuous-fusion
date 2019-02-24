#ifndef KITTIREADER_HPP
#define KITTIREADER_HPP

#include <vector>
#include <opencv2/opencv.hpp>

class KittiReader
{
    private:
    
    public:
        KittiReader(std::string basedir);

        static cv::Mat getPointcloud(cv::String fpath);
        static std::vector<cv::Mat> getImages(std::vector<cv::String> fpaths);
        static std::vector<cv::Mat> getPointclouds(std::vector<cv::String> fpaths);

        // Reading data
        static std::vector<cv::String> globFilepaths(std::string pattern);
        static std::map<std::string, std::vector<float>> getCalib(cv::String fpath);
        static void loadImagesAndPoints(std::string basedir);
        static std::vector<cv::String> globFilesHelper(std::string pattern);

        // Utils
        static std::vector<float> splitLineByChar(std::string line, char delim);
};

#endif