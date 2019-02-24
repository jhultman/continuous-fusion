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
        static void coutMatSize(cv::Mat mat);
        static void loadImagesAndPoints(std::string basedir);
        static std::vector<cv::String> globFilesHelper(std::string pattern);

        // Matrix utils
        static cv::Mat hconcatCol(cv::Mat matIn);
        static cv::Mat vconcatRow(cv::Mat matIn);
        static cv::Mat hconcatCol(cv::Mat matIn, cv::Mat col);
        static cv::Mat vconcatRow(cv::Mat matIn, cv::Mat row);

        // Matrix methods
        cv::Mat allAtOnce();
        cv::Mat getVeloToImagePRT();
        cv::Mat getVeloToCam0Unrect();
        cv::Mat getCam0RectToImage2();
        cv::Mat getCam0UnrectToCam2Rect();

        // Static matrix methods
        static cv::Mat getVeloToCam0Unrect(std::vector<float> R, std::vector<float> T);
        static cv::Mat getCam0UnrectToCam2Rect(std::vector<float> R_rect_00, std::vector<float> P_rect_02);
        static cv::Mat allAtOnce(cv::Mat RT, std::vector<float> R_rect_00, std::vector<float> P_rect_02);
        static cv::Mat getCam0RectToImage2(std::vector<float> P_rect_02);

    public:
        Calibration(std::string basedir);
        static std::vector<float> splitLineByChar(std::string line, char delim);
};

#endif
