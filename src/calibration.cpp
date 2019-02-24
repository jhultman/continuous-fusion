#include "calibration.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

Calibration::Calibration(
        std::map<std::string, std::vector<float>> calibVeloToCam,
        std::map<std::string, std::vector<float>> calibCamToCam)
{
    _calibVeloToCam = calibVeloToCam;
    _calibCamToCam = calibCamToCam;
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
    // 4x4 xform from velodyne coords to unrectified cam0 coords
    cv::Mat R3x3 = cv::Mat(R).reshape(1, 3);
    cv::Mat T3x1 = cv::Mat(T).reshape(1, 3);
    cv::Mat mat3x4 = Calibration::hconcatCol(R3x3, T3x1);
    cv::Mat mat4x4 = Calibration::vconcatRow(mat3x4);
    return mat4x4;
}

cv::Mat Calibration::getVeloToCam0Unrect()
{
    cv::Mat mat = getVeloToCam0Unrect(_calibVeloToCam.at("R"), _calibVeloToCam.at("T"));
    return mat;
}

cv::Mat Calibration::getCam0UnrectToCam2Rect(std::vector<float> R_rect_00, std::vector<float> P_rect_02)
{
    // 4x4 xform from unrectified cam0 coords to rectified cam2 coords
    cv::Mat R3x3 = cv::Mat(R_rect_00).reshape(1, 3);
    cv::Mat R3x4 = Calibration::hconcatCol(R3x3, cv::Mat::zeros(3, 1, CV_32F));
    cv::Mat R4x4 = Calibration::vconcatRow(R3x4);

    cv::Mat P3x4 = cv::Mat(P_rect_02).reshape(1, 3);
    cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
    T2.at<float>(0, 3) = P3x4.at<float>(0, 3) / P3x4.at<float>(0, 0);
    cv::Mat mat = T2.mul(R4x4);
    return mat;
}

cv::Mat Calibration::getCam0UnrectToCam2Rect()
{
    cv::Mat mat = getCam0UnrectToCam2Rect(_calibCamToCam.at("R_rect_00"), _calibCamToCam.at("P_rect_02"));
    return mat;
}

cv::Mat Calibration::allAtOnce(cv::Mat RT, std::vector<float> R_rect_00, std::vector<float> P_rect_02)
{
    // 4x4 xform from unrectified cam0 coords to rectified cam2 coords
    cv::Mat R3x3 = cv::Mat(R_rect_00).reshape(1, 3);
    cv::Mat R3x4 = Calibration::hconcatCol(R3x3, cv::Mat::zeros(3, 1, CV_32F));
    cv::Mat R4x4 = Calibration::vconcatRow(R3x4);
    cv::Mat P3x4 = cv::Mat(P_rect_02).reshape(1, 3);
    cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
    T2.at<float>(0, 3) = P3x4.at<float>(0, 3) / P3x4.at<float>(0, 0);
    cv::Mat mat = T2.mul(R4x4).mul(RT);
    return mat;
}

cv::Mat Calibration::allAtOnce()
{
    auto RT = getVeloToCam0Unrect();
    std::cout << RT << std::endl << std::endl;
    cv::Mat mat = allAtOnce(RT, _calibCamToCam.at("R_rect_00"), _calibCamToCam.at("P_rect_02"));
    return mat;
}

cv::Mat Calibration::getCam0RectToImage2(std::vector<float> P_rect_02)
{
    // Projection from rectified cam0 coords to image plane of cam2
    cv::Mat mat3x4 = cv::Mat(P_rect_02).reshape(1, 3);
    cv::Mat mat4x4 = Calibration::vconcatRow(mat3x4);
    return mat4x4;
}

cv::Mat Calibration::getCam0RectToImage2()
{
    cv::Mat mat = getCam0RectToImage2(_calibCamToCam.at("P_rect_02"));
    return mat;
}

cv::Mat Calibration::getVeloToImagePRT()
{
    // x_image = P * R * T * x_velo as given in (7) in Geiger, et al.
    cv::Mat cam0RectToImage2 = getCam0RectToImage2();
    cv::Mat veloToCam0Unrect = getVeloToCam0Unrect();
    cv::Mat cam0UnrectToCam2Rect = getCam0UnrectToCam2Rect();

    cv::Mat P = cam0RectToImage2;
    cv::Mat R = veloToCam0Unrect.inv();
    cv::Mat T = allAtOnce().inv();
    
    cv::Mat PRT = P.mul(R).mul(T);
    std::cout << T << std::endl << std::endl;
    return PRT;
}

void Calibration::coutMatSize(cv::Mat mat)
{
    std::cout << "[" << mat.rows << " x " << mat.cols << "]" << std::endl;
}