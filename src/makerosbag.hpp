#ifndef MAKEROSBAG_HPP
#define MAKEROSBAG_HPP

#include <vector>
#include <opencv2/opencv.hpp>

std::vector<cv::String> getFilepaths(std::string pattern);
std::vector<cv::Mat> getImages(std::vector<cv::String> fpaths);
std::vector<cv::Vec4f> getPointcloud(cv::String fpath);
std::vector<std::vector<cv::Vec4f>> getPointclouds(std::vector<cv::String> fpaths);
std::vector<float> splitLineByChar(std::string line, char delim);
std::map<std::string, std::vector<float>> getCalib(cv::String fpath);

#endif
