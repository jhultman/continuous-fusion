#include "makerosbag.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

Calibration::Calibration(std::string basedir)
{
    _calibVeloToCam = Calibration::getCalib(basedir + "calib_velo_to_cam.txt");
    _calibCamToCam = Calibration::getCalib(basedir + "calib_cam_to_cam.txt");
    auto PRT = getVeloToImagePRT();
}

std::vector<cv::String> Calibration::globFilesHelper(std::string pattern)
{
    std::vector<cv::String> fpaths; 
    cv::glob(pattern, fpaths, false);
    return fpaths;
}

std::vector<cv::Mat> Calibration::getImages(std::vector<cv::String> fpaths)
{
    std::vector<cv::Mat> images;
    for(auto const& fpath : fpaths) {
        auto image = cv::imread(fpath, CV_LOAD_IMAGE_COLOR);
        images.push_back(image);
    }
    return images;
}

cv::Mat Calibration::getPointcloud(cv::String fpath)
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

std::vector<cv::Mat> Calibration::getPointclouds(std::vector<cv::String> fpaths)
{
    std::vector<cv::Mat> scans;
    for(auto const& fpath : fpaths) {
        auto scan = getPointcloud(fpath);
        scans.push_back(scan);
    }
    return scans;
}

std::vector<float> Calibration::splitLineByChar(std::string line, char delim)
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

cv::Mat Calibration::vconcatRow(cv::Mat matIn, cv::Mat row)
{
    assert (matIn.cols == row.cols);
    size_t rows = matIn.rows + 1;
    size_t cols = matIn.cols;
    cv::Mat matOut(rows, cols, CV_32F);
    cv::vconcat(matIn, row, matOut);
    return matOut;
}

cv::Mat Calibration::hconcatCol(cv::Mat matIn, cv::Mat col)
{
    assert (matIn.rows == col.rows);
    size_t rows = matIn.rows;
    size_t cols = matIn.cols + 1;
    cv::Mat matOut(rows, cols, CV_32F);
    cv::hconcat(matIn, col, matOut);
    return matOut;
}

cv::Mat Calibration::vconcatRow(cv::Mat matIn)
{
    assert (matIn.rows == 3);
    assert (matIn.cols == 4);
    cv::Mat row = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::Mat matOut = Calibration::vconcatRow(matIn, row);
    return matOut;
}

cv::Mat Calibration::hconcatCol(cv::Mat matIn)
{
    assert (matIn.rows == 4);
    assert (matIn.cols == 3);
    cv::Mat col = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    cv::Mat matOut = Calibration::hconcatCol(matIn, col);
    return matOut;
}

std::map<std::string, std::vector<float>> Calibration::getCalib(cv::String fpath)
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

cv::Mat Calibration::getVeloToCam0Unrect()
{
    // 4x4 xform from velodyne coords to unrectified cam0 coords
    cv::Mat R3x3 = cv::Mat(_calibVeloToCam.at("R")).reshape(1, 3);
    cv::Mat T3x1 = cv::Mat(_calibVeloToCam.at("T")).reshape(1, 3);
    cv::Mat mat3x4 = Calibration::hconcatCol(R3x3, T3x1);
    cv::Mat mat4x4 = Calibration::vconcatRow(mat3x4);
    return mat4x4;
}

cv::Mat Calibration::getCam0UnrectToCam2Rect()
{
    // 4x4 xform from unrectified cam0 coords to rectified cam2 coords
    cv::Mat R3x3 = cv::Mat(_calibCamToCam.at("R_rect_00")).reshape(1, 3);
    cv::Mat R3x4 = Calibration::hconcatCol(R3x3, cv::Mat::zeros(3, 1, CV_32F));
    cv::Mat R4x4 = Calibration::vconcatRow(R3x4);

    cv::Mat P3x4 = cv::Mat(_calibCamToCam.at("P_rect_02")).reshape(1, 3);
    cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
    T2.at<float>(0, 3) = P3x4.at<float>(0, 3) / P3x4.at<float>(0, 0);
    cv::Mat mat = T2.mul(R4x4);
    return mat;
}

cv::Mat Calibration::allAtOnce()
{
    // 4x4 xform from unrectified cam0 coords to rectified cam2 coords
    auto RT = getVeloToCam0Unrect();
    std::cout << RT << std::endl << std::endl;

    cv::Mat R3x3 = cv::Mat(_calibCamToCam.at("R_rect_00")).reshape(1, 3);
    cv::Mat R3x4 = Calibration::hconcatCol(R3x3, cv::Mat::zeros(3, 1, CV_32F));
    cv::Mat R4x4 = Calibration::vconcatRow(R3x4);
    cv::Mat P3x4 = cv::Mat(_calibCamToCam.at("P_rect_02")).reshape(1, 3);
    cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
    T2.at<float>(0, 3) = P3x4.at<float>(0, 3) / P3x4.at<float>(0, 0);
    cv::Mat mat = T2.mul(R4x4).mul(RT);
    return mat;
}

cv::Mat Calibration::getCam0RectToImage2()
{
    // Projection from rectified cam0 coords to image plane of cam2
    cv::Mat mat3x4 = cv::Mat(_calibCamToCam.at("P_rect_02")).reshape(1, 3);
    cv::Mat mat4x4 = Calibration::vconcatRow(mat3x4);
    return mat4x4;
}

cv::Mat Calibration::getVeloToImagePRT()
{
    // x_image = P * R * T * x_velo as given in (7) in Geiger, et al.
    cv::Mat cam0RectToImage2 = getCam0RectToImage2();
    cv::Mat veloToCam0Unrect = getVeloToCam0Unrect();
    cv::Mat cam0UnrectToCam2Rect = getCam0UnrectToCam2Rect();

    cv::Mat P = cam0RectToImage2;
    cv::Mat R = veloToCam0Unrect.inv(); //cam0unrecttovelo
    cv::Mat T = allAtOnce().inv();
    
    cv::Mat PRT = P.mul(R).mul(T);
    std::cout << T << std::endl << std::endl;
    return PRT;
}

void Calibration::loadImagesAndPoints(std::string basedir)
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

void Calibration::coutMatSize(cv::Mat mat)
{
    std::cout << "[" << mat.rows << " x " << mat.cols << "]" << std::endl;
}

int main(int argc, const char* argv[])
{
    std::string basedir = argv[1];
    Calibration calib = Calibration(basedir);
    return 0;
}
