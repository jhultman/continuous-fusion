#ifndef MAKEROSBAG_HPP
#define MAKEROSBAG_HPP

#include <vector>
#include <opencv2/opencv.hpp>

class Calibration
{

    private:
        std::map<std::string, std::vector<float>> _calibVeloToCam;
        std::map<std::string, std::vector<float>> _calibCamToCam;

        // Reading data
        static cv::Mat getPointcloud(cv::String fpath);
        static std::vector<cv::Mat> getImages(std::vector<cv::String> fpaths);
        static std::vector<cv::Mat> getPointclouds(std::vector<cv::String> fpaths);
        static std::vector<cv::String> globFilepaths(std::string pattern);
        static std::map<std::string, std::vector<float>> getCalib(cv::String fpath);

        // Utils
        static std::vector<cv::String> globFilesHelper(std::string pattern);
        static void loadImagesAndPoints(std::string basedir);
        static void coutMatSize(cv::Mat mat);

        // Matrix methods
        cv::Mat allAtOnce();
        cv::Mat getVeloToImagePRT();
        cv::Mat getVeloToCam0Unrect();
        cv::Mat getCam0RectToImage2();
        cv::Mat getCam0UnrectToCam2Rect();

    public:
        Calibration(std::string basedir);
        static std::vector<float> splitLineByChar(std::string line, char delim);
};

#endif
