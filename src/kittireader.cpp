#include "kittireader.hpp"
#include "calibration.hpp"
#include <vector>
#include <opencv2/opencv.hpp>

KittiReader::KittiReader(std::string basedir)
{
    auto calib = makeCalib(basedir);
    auto PRT = calib.getVeloToImage2();
}

std::vector<cv::String> KittiReader::globFilesHelper(std::string pattern)
{
    std::vector<cv::String> fpaths; 
    cv::glob(pattern, fpaths, false);
    return fpaths;
}

std::vector<cv::Mat> KittiReader::getImages(std::vector<cv::String> fpaths)
{
    std::vector<cv::Mat> images;
    for(auto const& fpath : fpaths) {
        auto image = cv::imread(fpath, CV_LOAD_IMAGE_COLOR);
        images.push_back(image);
    }
    return images;
}

cv::Mat KittiReader::getPointcloud(cv::String fpath)
{
    cv::Vec4f point;
    std::vector<cv::Vec4f> points;
    std::ifstream ifs(fpath.c_str(), std::ios::in | std::ios::binary);
    while(ifs.good())
    {
        ifs.read((char *) &point, 4*sizeof(float));
        points.push_back(point);
    }
    cv::Mat mat(points);
    return mat;
}

std::vector<cv::Mat> KittiReader::getPointclouds(std::vector<cv::String> fpaths)
{
    std::vector<cv::Mat> scans;
    for(auto const& fpath : fpaths) {
        auto scan = getPointcloud(fpath);
        scans.push_back(scan);
    }
    return scans;
}

std::vector<float> KittiReader::splitLineByChar(std::string line, char delim)
{
    std::string val;
    std::vector<float> vals;
    std::istringstream iss(line);
    while(std::getline(iss, val, delim))
    {
        vals.push_back(atof(val.c_str()));
    }
    return vals;
}

void KittiReader::loadImagesAndPoints(std::string basedir)
{
    std::string patternImages = basedir + 
        "2011_09_26_drive_0005_sync/image_02/data/*.png";
    std::string patternPoints = basedir + 
        "2011_09_26_drive_0005_sync/velodyne_points/data/*.bin";
    auto imageFpaths = globFilesHelper(patternImages);
    auto pointcloudFpaths = globFilesHelper(patternPoints);
    auto images = getImages(imageFpaths);
    auto points = getPointclouds(pointcloudFpaths);
    return;
}

std::map<std::string, std::vector<float>> KittiReader::getCalib(cv::String fpath)
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

Calibration KittiReader::makeCalib(cv::String basedir)
{
    auto veloToCam = getCalib(basedir + "calib_velo_to_cam.txt");
    auto camToCam = getCalib(basedir + "calib_cam_to_cam.txt");
    auto calib = Calibration(veloToCam, camToCam);
    return calib;
}