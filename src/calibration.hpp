#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <vector>
#include <opencv2/opencv.hpp>

class Calibration
{
    private:
        std::map<std::string, std::vector<float>> _calibVeloToCam;
        std::map<std::string, std::vector<float>> _calibCamToCam;

        // Matrix utils
        static void coutMatSize(cv::Mat mat);
        static cv::Mat hconcatCol(cv::Mat matIn);
        static cv::Mat vconcatRow(cv::Mat matIn);
        static cv::Mat hconcatCol(cv::Mat matIn, cv::Mat col);
        static cv::Mat vconcatRow(cv::Mat matIn, cv::Mat row);

        // Static matrix methods
        static cv::Mat allAtOnce(cv::Mat RT, std::vector<float> R_rect_00, std::vector<float> P_rect_02);
        static cv::Mat getVeloToCam0Unrect(std::vector<float> R, std::vector<float> T);
        static cv::Mat getCam0RectToImage2(std::vector<float> P_rect_02);
        static cv::Mat getCam0UnrectToCam2Rect(std::vector<float> R_rect_00, std::vector<float> P_rect_02);

        // Matrix methods
        cv::Mat allAtOnce();
        cv::Mat getVeloToCam0Unrect();
        cv::Mat getCam0RectToImage2();
        cv::Mat getCam0UnrectToCam2Rect();

    public:
        Calibration(
            std::map<std::string, std::vector<float>> _calibVeloToCam,
            std::map<std::string, std::vector<float>> _calibCamToCam);

        cv::Mat getVeloToImagePRT();
};

#endif