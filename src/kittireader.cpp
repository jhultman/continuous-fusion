#include "kittireader.hpp"
#include "calibration.hpp"
#include <vector>
#include <opencv2/opencv.hpp>
#include "pcl_ros/point_cloud.h"

KittiReader::KittiReader(std::string basedir)
{
    auto calib = makeCalib(basedir);
    auto PRT = calib.getVeloToImage();
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
    for(auto const& fpath : fpaths) 
    {
        auto image = cv::imread(fpath, CV_LOAD_IMAGE_COLOR);
        images.push_back(image);
    }
    return images;
}

pcl::PointXYZI PointXYZI_(cv::Vec4f fields)
{ 
    pcl::PointXYZI pt;
    pt.x = fields[0]; 
    pt.y = fields[1]; 
    pt.z = fields[2]; 
    pt.intensity = fields[3]; 
    return pt;
}

pcl::PointCloud<pcl::PointXYZI> KittiReader::getPointcloud(cv::String fpath)
{
    // Points outside approximate camera viewing frustum filtered.
    std::ifstream ifs(fpath.c_str(), std::ios::in | std::ios::binary);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->is_dense = false;
    cv::Vec4f point;
    while(ifs.good())
    {
        ifs.read((char *) &point, 4*sizeof(float));
        if (
            (point[0] > 0) & 
            (abs(point[0]) > abs(point[1])) & 
            (point[0] > 0) & 
            (point[0] < 48) & 
            (point[1] > -24) &
            (point[1] < 24))
        {
            cloud->push_back(PointXYZI_(point));
        }
    }
    return *cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>> KittiReader::getPointclouds(std::vector<cv::String> fpaths)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds;
    for(auto const& fpath : fpaths) 
    {
        cloud = getPointcloud(fpath);
        clouds.push_back(cloud);
    }
    return clouds;
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
