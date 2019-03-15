#include "calibration.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

Calibration::Calibration(
        std::map<std::string, std::vector<float>> veloToCam,
        std::map<std::string, std::vector<float>> camToCam)
{
    _veloToCam = veloToCam;
    _camToCam = camToCam;
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

cv::Mat Calibration::getVeloToCam0Unrect(std::vector<float> R, std::vector<float> T)
{
    cv::Mat R3x3 = cv::Mat(R).reshape(1, 3);
    cv::Mat T3x1 = cv::Mat(T).reshape(1, 3);
    cv::Mat mat3x4 = Calibration::hconcatCol(R3x3, T3x1);
    cv::Mat mat4x4 = Calibration::vconcatRow(mat3x4);
    return mat4x4;
}

cv::Mat Calibration::getCam0UnrectToCam0Rect(std::vector<float> R_rect_00)
{
    cv::Mat R3x3 = cv::Mat(R_rect_00).reshape(1, 3);
    cv::Mat R3x4 = Calibration::hconcatCol(R3x3, cv::Mat::zeros(3, 1, CV_32F));
    cv::Mat R4x4 = Calibration::vconcatRow(R3x4);
    return R4x4;
}

cv::Mat Calibration::getCam0RectToImage2(std::vector<float> P_rect_02)
{
    cv::Mat mat3x4 = cv::Mat(P_rect_02).reshape(1, 3);
    cv::Mat mat4x4 = Calibration::vconcatRow(mat3x4);
    return mat4x4;
}

cv::Mat Calibration::getVeloToCam0Unrect()
{
    cv::Mat mat = getVeloToCam0Unrect(_veloToCam.at("R"), _veloToCam.at("T"));
    return mat;
}

cv::Mat Calibration::getCam0UnrectToCam0Rect()
{
    cv::Mat mat = getCam0UnrectToCam0Rect(_camToCam.at("R_rect_00"));
    return mat;
}

cv::Mat Calibration::getCam0RectToImage2()
{
    cv::Mat mat = getCam0RectToImage2(_camToCam.at("P_rect_02"));
    return mat;
}

cv::Mat Calibration::getVeloToImage2()
{
    /* 
    Following (7) in Geiger et al., points x_velo 
    of form (x, y, z, 1) in velodyne coords are sent 
    to points x_image of form (u, v, 1) in the image 
    plane of camera 2 using: 

        x_image = P * R * T * x_velo, 
        
    where:

        x_velo  - point in velodyne coords (x, y, z, 1).
        T       - velodyne coords to unrectified cam0 coords.
        R       - unrectified cam0 coords to rectified cam 0 coords,
        P       - rectified cam0 coords to image plane of cam2,
        x_image - point in image of cam2 (u, v, 1).

    Note that in this module we use left-multiplying active 
    (alibi) coordinate transformations to maintain consistency 
    with KITTI. ROS uses the opposite passive (alias) 
    transformation convention.
    */
    auto T = getVeloToCam0Unrect();
    auto R = getCam0UnrectToCam0Rect();
    auto P = getCam0RectToImage2();
    auto PRT = P * R * T;
    return PRT;
}