#include "makerosbag.hpp"
#include <vector>
#include <opencv2/opencv.hpp>

std::vector<cv::String> globFilepaths(std::string pattern)
{
    std::vector<cv::String> fpaths; 
    cv::glob(pattern, fpaths, false);
    return fpaths;
}

std::vector<cv::Mat> getImages(std::vector<cv::String> fpaths)
{
    std::vector<cv::Mat> images;
    for(auto const& fpath : fpaths) {
        auto image = cv::imread(fpath, CV_LOAD_IMAGE_COLOR);
        images.push_back(image);
    }
    return images;
}

cv::Mat getPointcloud(cv::String fpath)
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

std::vector<cv::Mat> getPointclouds(std::vector<cv::String> fpaths)
{
    std::vector<cv::Mat> scans;
    for(auto const& fpath : fpaths) {
        auto scan = getPointcloud(fpath);
        scans.push_back(scan);
    }
    return scans;
}

std::vector<float> splitLineByChar(std::string line, char delim)
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

cv::Mat padThreeByFour(cv::Mat mat)
{
    cv::Mat catMat(4, 4, CV_32F);
    cv::Mat row = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::vconcat(mat, row, catMat);
    return catMat;
}

cv::Mat vectorToMatFourByFour(std::vector<float> vals)
{
    cv::Mat valsMat = cv::Mat(vals).reshape(0, 3);
    auto padded = padThreeByFour(valsMat);
    return padded;
}

int main(int argc, const char* argv[])
{
    std::string basedir = argv[1];
    std::string patternImages = basedir + 
        "2011_09_26_drive_0005_sync/image_02/data/*.png";
    std::string patternPoints = basedir + 
        "2011_09_26_drive_0005_sync/velodyne_points/data/*.bin";
    auto imageFpaths = globFilepaths(patternImages);
    auto pointcloudFpaths = globFilepaths(patternPoints);
    auto images = getImages(imageFpaths);
    auto points = getPointclouds(pointcloudFpaths);
    auto calibCamToCam = getCalib(basedir + "calib_cam_to_cam.txt");
    auto calibVeloToCam = getCalib(basedir + "calib_velo_to_cam.txt");
    auto p2 = vectorToMatFourByFour(calibCamToCam.at("P_rect_02"));
    return 0;
}
