#ifndef MAKEROSBAG_HPP
#define MAKEROSBAG_HPP

#include <vector>
#include <opencv2/opencv.hpp>

// Reading data
std::vector<cv::String> globFilepaths(std::string pattern);
cv::Mat getPointcloud(cv::String fpath);
std::vector<cv::Mat> getPointclouds(std::vector<cv::String> fpaths);
std::vector<cv::Mat> getImages(std::vector<cv::String> fpaths);

// Utils
std::vector<float> splitLineByChar(std::string line, char delim);
std::map<std::string, std::vector<float>> getCalib(cv::String fpath);
void loadImagesAndPoints(std::string basedir);
void coutMatSize(cv::Mat mat);

// calibration matrices
cv::Mat getVeloToCam0Unrect(std::map<std::string, std::vector<float>> calib);
cv::Mat getCam0UnrectToCam2Rect(std::map<std::string, std::vector<float>> calib);
cv::Mat allAtOnce(
        std::map<std::string, std::vector<float>> calibVeloToCam,
        std::map<std::string, std::vector<float>> calibCamToCam);
cv::Mat getCam0RectToImage2(std::map<std::string, std::vector<float>> calib);
cv::Mat getVeloToImagePRT(
        std::map<std::string, std::vector<float>> calibCamToCam,
        std::map<std::string, std::vector<float>> calibVeloToCam);

#endif
