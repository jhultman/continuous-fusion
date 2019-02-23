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

cv::Mat getT_cam0_velo_unrect(std::map<std::string, std::vector<float>> calib)
{
    // 4x4 xform from velodyne coordinates to unrectified cam0 coords
    cv::Mat R3x3 = cv::Mat(calib.at("R")).reshape(1, 3);
    cv::Mat T3x1 = cv::Mat(calib.at("T")).reshape(1, 3);
    cv::Mat row1x4 = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::Mat RT3x4(3, 4, CV_32F);
    cv::Mat RT4x4(4, 4, CV_32F);
    cv::hconcat(R3x3, T3x1, RT3x4);
    cv::vconcat(RT3x4, row1x4, RT4x4);
    return RT4x4;
}

cv::Mat getP_rect_00(std::map<std::string, std::vector<float>> calib)
{
    // 3x4 projection matrix
    cv::Mat P_rect_00 = cv::Mat(calib.at("P_rect_00")).reshape(1, 3);
    return P_rect_00;
}

cv::Mat getT2(std::map<std::string, std::vector<float>> calib)
{
    // Rectified extrinsics from cam0 to cam2
    cv::Mat P_rect_02 = cv::Mat(calib.at("P_rect_02")).reshape(1, 3);
    cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
    T2.at<float>(3, 3) = P_rect_02.at<float>(0, 3) / P_rect_02.at<float>(0, 0);
    return T2;
}

cv::Mat getR_rect_00(std::map<std::string, std::vector<float>> calib)
{
    cv::Mat R_rect_00 = cv::Mat(calib.at("R_rect_00")).reshape(1, 3);
    cv::Mat col3x1 = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    cv::Mat row1x4 = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::Mat mat3x4(3, 4, CV_32F);
    cv::Mat mat4x4(4, 4, CV_32F);
    cv::hconcat(R_rect_00, col3x1, mat3x4);
    cv::vconcat(mat3x4, row1x4, mat4x4);
    return mat4x4;
}

void loadImagesAndPoints(std::string basedir)
{
    std::string patternImages = basedir + 
        "2011_09_26_drive_0005_sync/image_02/data/*.png";
    std::string patternPoints = basedir + 
        "2011_09_26_drive_0005_sync/velodyne_points/data/*.bin";
    auto imageFpaths = globFilepaths(patternImages);
    auto pointcloudFpaths = globFilepaths(patternPoints);
    auto images = getImages(imageFpaths);
    auto points = getPointclouds(pointcloudFpaths);
    return;
}

void coutMatSize(cv::Mat mat)
{
    std::cout << "[" << mat.rows << " x " << mat.cols << "]" << std::endl;
}

void loadCalib(std::string basedir)
{
    auto calibCamToCam = getCalib(basedir + "calib_cam_to_cam.txt");
    auto calibVeloToCam = getCalib(basedir + "calib_velo_to_cam.txt");
    auto unrect = getT_cam0_velo_unrect(calibVeloToCam);
    auto R0_rect = unrect.inv(cv::DECOMP_LU);

    auto P_rect_00 = getP_rect_00(calibCamToCam);
    auto T2 = getT2(calibCamToCam);
    auto R_rect_00 = getR_rect_00(calibCamToCam);

    auto T_cam2_to_velo = T2.mul(R_rect_00.mul(unrect));
    auto T_velo_to_cam2 = T_cam2_to_velo.inv(cv::DECOMP_LU);
    return;
}

int main(int argc, const char* argv[])
{
    std::string basedir = argv[1];
    loadCalib(basedir);
    return 0;
}
