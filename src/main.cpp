#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>


std::vector<cv::String> getFilepaths(std::string pattern)
{
    std::vector<cv::String> fpaths; 
    cv::glob(pattern, fpaths, false);
    return fpaths;
}


std::vector<cv::Mat> getImages(std::vector<cv::String> fpaths)
{
    std::vector<cv::Mat> images;
    for(auto const& fpath: fpaths) {
        auto image = cv::imread(fpath, CV_LOAD_IMAGE_COLOR);
        images.push_back(image);
    }
    return images;
}


std::vector<cv::Vec4f> getPointcloud(cv::String fpath)
{
    cv::Vec4f point;
    std::vector<cv::Vec4f> points;

    std::ifstream ifs(fpath.c_str(), std::ios::in | std::ios::binary);
    while(ifs.good())
    {
        ifs.read((char *) &point, 4*sizeof(float));
        points.push_back(point);
    }
    return points;
}


std::vector<std::vector<cv::Vec4f>> getPointclouds(std::vector<cv::String> fpaths)
{
    std::vector<std::vector<cv::Vec4f>> scans;
    for(auto const& fpath: fpaths) {
        auto scan = getPointcloud(fpath);
        scans.push_back(scan);
    }
    return scans;
}


int main(int argc, const char* argv[])
{
    std::string basedir = argv[1];
    std::string patternImages = basedir + "image_02/data/*.png";
    std::string patternPoints = basedir + "velodyne_points/data/*.bin";

    auto imageFpaths = getFilepaths(patternImages);
    auto images = getImages(imageFpaths);

    auto pointcloudFpaths = getFilepaths(patternPoints);
    auto points = getPointclouds(pointcloudFpaths);
    return 0;
}