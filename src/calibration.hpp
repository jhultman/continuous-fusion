#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <vector>
#include <opencv2/opencv.hpp>

class Calibration
{
    private:
        std::map<std::string, std::vector<float>> _veloToCam;
        std::map<std::string, std::vector<float>> _camToCam;

        // Matrix utils
        static cv::Mat hconcatCol(cv::Mat matIn);
        static cv::Mat vconcatRow(cv::Mat matIn);
        static cv::Mat hconcatCol(cv::Mat matIn, cv::Mat col);
        static cv::Mat vconcatRow(cv::Mat matIn, cv::Mat row);

    public:
        Calibration(
            std::map<std::string, std::vector<float>> veloToCam,
            std::map<std::string, std::vector<float>> camToCam);

        cv::Mat getVeloToCam0Unrect();
        cv::Mat getCam0UnrectToCam0Rect();
        cv::Mat getCam0RectToImage2();
        cv::Mat getVeloToImage2();
};

#endif