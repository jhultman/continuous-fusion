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
cv::Mat padThreeByFour(cv::Mat mat);
std::map<std::string, std::vector<float>> getCalib(cv::String fpath);
void coutMatSize(cv::Mat mat);

// calibration matrices
cv::Mat getT2(std::map<std::string, std::vector<float>> calib);
cv::Mat getP_rect_00(std::map<std::string, std::vector<float>> calib);
cv::Mat getT_cam0_velo_unrect(std::map<std::string, std::vector<float>> calib);
cv::Mat getT2(std::map<std::string, std::vector<float>> calib);
cv::Mat getR_rect_00(std::map<std::string, std::vector<float>> calib);

#endif
