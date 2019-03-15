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
        static void coutMatSize(cv::Mat mat);
        static cv::Mat hconcatCol(cv::Mat matIn);
        static cv::Mat vconcatRow(cv::Mat matIn);
        static cv::Mat hconcatCol(cv::Mat matIn, cv::Mat col);
        static cv::Mat vconcatRow(cv::Mat matIn, cv::Mat row);

        // Matrix methods
        cv::Mat getVeloToCam0Unrect();
        cv::Mat getCam0UnrectToCam0Rect();
        cv::Mat getCam0RectToImage2();

    public:
        Calibration(
            std::map<std::string, std::vector<float>> veloToCam,
            std::map<std::string, std::vector<float>> camToCam);
        
        static cv::Mat getVeloToCam0Unrect(std::vector<float> R, std::vector<float> T);
        static cv::Mat getCam0UnrectToCam0Rect(std::vector<float> R_rect_00);
        static cv::Mat getCam0RectToImage2(std::vector<float> P_rect_02);
        cv::Mat getVeloToImage2();
};

#endif