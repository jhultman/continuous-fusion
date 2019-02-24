#include "makerosbag.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

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

cv::Mat getVeloToCam0Unrect(std::map<std::string, std::vector<float>> calib)
{
    // 4x4 xform from velodyne coords to unrectified cam0 coords
    cv::Mat R3x3 = cv::Mat(calib.at("R")).reshape(1, 3);
    cv::Mat T3x1 = cv::Mat(calib.at("T")).reshape(1, 3);
    cv::Mat row1x4 = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::Mat mat3x4(3, 4, CV_32F);
    cv::Mat mat4x4(4, 4, CV_32F);
    cv::hconcat(R3x3, T3x1, mat3x4);
    cv::vconcat(mat3x4, row1x4, mat4x4);
    return mat4x4;
}

cv::Mat getCam0UnrectToCam2Rect(std::map<std::string, std::vector<float>> calib)
{
    // 4x4 xform from unrectified cam0 coords to rectified cam2 coords
    cv::Mat R3x3 = cv::Mat(calib.at("R_rect_00")).reshape(1, 3);
    cv::Mat R3x4(3, 4, CV_32F);
    cv::Mat R4x4(4, 4, CV_32F);
    cv::Mat col3x1 = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    cv::Mat row1x4 = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::hconcat(R3x3, col3x1, R3x4);
    cv::vconcat(R3x4, row1x4, R4x4);
    cv::Mat P3x4 = cv::Mat(calib.at("P_rect_02")).reshape(1, 3);
    cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
    T2.at<float>(0, 3) = P3x4.at<float>(0, 3) / P3x4.at<float>(0, 0);
    cv::Mat mat = T2.mul(R4x4);
    return mat;
}

cv::Mat allAtOnce(
        std::map<std::string, std::vector<float>> calibVeloToCam,
        std::map<std::string, std::vector<float>> calibCamToCam)
{
    // 4x4 xform from unrectified cam0 coords to rectified cam2 coords
    
    auto RT = getVeloToCam0Unrect(calibVeloToCam);
    std::cout << RT << std::endl << std::endl;

    cv::Mat R3x3 = cv::Mat(calibCamToCam.at("R_rect_00")).reshape(1, 3);
    cv::Mat R3x4(3, 4, CV_32F);
    cv::Mat R4x4(4, 4, CV_32F);
    cv::Mat col3x1 = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    cv::Mat row1x4 = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::hconcat(R3x3, col3x1, R3x4);
    cv::vconcat(R3x4, row1x4, R4x4);
    cv::Mat P3x4 = cv::Mat(calibCamToCam.at("P_rect_02")).reshape(1, 3);
    cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
    T2.at<float>(0, 3) = P3x4.at<float>(0, 3) / P3x4.at<float>(0, 0);
    cv::Mat mat = T2.mul(R4x4).mul(RT);
    return mat;
}

cv::Mat getCam0RectToImage2(std::map<std::string, std::vector<float>> calib)
{
    // Projection from rectified cam0 coords to image plane of cam2
    cv::Mat mat3x4 = cv::Mat(calib.at("P_rect_02")).reshape(1, 3);
    cv::Mat row1x4 = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::Mat mat4x4(4, 4, CV_32F);
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

cv::Mat getVeloToImagePRT(std::map<std::string, std::vector<float>> calibCamToCam, std::map<std::string, std::vector<float>> calibVeloToCam)
{
    // x_image = P * R * T * x_velo as given in (7) in Geiger, et al.
    cv::Mat cam0RectToImage2 = getCam0RectToImage2(calibCamToCam);
    cv::Mat veloToCam0Unrect = getVeloToCam0Unrect(calibVeloToCam);
    cv::Mat cam0UnrectToCam2Rect = getCam0UnrectToCam2Rect(calibCamToCam);

    cv::Mat P = cam0RectToImage2;
    cv::Mat R = veloToCam0Unrect.inv(); //cam0unrecttovelo
    cv::Mat T = (allAtOnce(calibVeloToCam, calibCamToCam)).inv();
    
    cv::Mat PRT = P.mul(R).mul(T);
    std::cout << T << std::endl << std::endl;
    return PRT;
}

int main(int argc, const char* argv[])
{
    std::string basedir = argv[1];
    auto calibCamToCam = getCalib(basedir + "calib_cam_to_cam.txt");
    auto calibVeloToCam = getCalib(basedir + "calib_velo_to_cam.txt");
    auto PRT = getVeloToImagePRT(calibCamToCam, calibVeloToCam);
    return 0;
}
