#ifndef MAKEROSBAG_HPP
#define MAKEROSBAG_HPP

#include <vector>
#include <opencv2/opencv.hpp>

std::vector<cv::String> globFilepaths(std::string pattern);
std::vector<cv::Mat> getImages(std::vector<cv::String> fpaths);
cv::Mat getPointcloud(cv::String fpath);
std::vector<cv::Mat> getPointclouds(std::vector<cv::String> fpaths);
std::vector<float> splitLineByChar(std::string line, char delim);
std::map<std::string, std::vector<float>> getCalib(cv::String fpath);

#endif
