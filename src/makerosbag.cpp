#include "makerosbag.hpp"
#include <vector>
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


std::vector<float> splitLineByChar(std::string line, char delim)
{
    std::istringstream iss(line);
    std::string val;
    std::vector<float> vals;
    while(std::getline(iss, val, delim))
    {
        vals.push_back(atof(val.c_str()));
    }
    return vals;
}


std::map<std::string, std::vector<float>> getCalib(cv::String fpath)
{
    std::map<std::string, std::vector<float>> calib;
    std::ifstream ifs(fpath.c_str());
    std::string key, line;

    while(ifs.good())
    {
        std::getline(ifs, key, ':');
        ifs.ignore(1, ' ');
        std::getline(ifs, line, '\n');
        auto val = splitLineByChar(line, ' ');
        calib.insert(std::make_pair(key, val));
    }
    return calib;
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

    std::string calibFname = basedir + "calib_cam_to_cam.txt";
    auto calib = getCalib(calibFname);
    return 0;
}
