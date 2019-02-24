#include "catch.hpp"
#include "calibration.hpp"
#include "kittireader.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

/* LHS computed using pykitti. */

bool areEqual(cv::Mat mat1, cv::Mat mat2)
{
    if (mat1.size != mat2.size)
    {
        return false;
    }
    cv::Mat diff;
    cv::compare(mat1, mat2, diff, cv::CMP_NE);
    return (cv::countNonZero(diff) == 0);
}

TEST_CASE( "veloToCam0Unrect", "[xforms]" )
{
    std::string strExpected = "0.007533745 -0.9999714 -0.000616602 -0.004069766 0.01480249 0.0007280733 -0.9998902 -0.07631618 0.9998621 0.00752379 0.01480755 -0.2717806 0.0 0.0 0.0 1.0";
    std::string strR = "0.007533745 -0.9999714 -0.000616602 0.01480249 0.0007280733 -0.9998902 0.9998621 0.00752379 0.01480755";
    std::string strT = "-0.004069766 -0.07631618 -0.2717806";

    auto R = KittiReader::splitLineByChar(strR, ' ');
    auto T = KittiReader::splitLineByChar(strT, ' ');
    auto vecExpected = KittiReader::splitLineByChar(strExpected, ' ');

    auto expected = cv::Mat(vecExpected).reshape(1, 4);
    auto actual = Calibration::getVeloToCam0Unrect(R, T);
    REQUIRE(areEqual(expected, actual));
}

TEST_CASE( "cam0RectToImage2", "[xforms]" )
{
    std::string strP_rect_02 = "7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01 0.000000e+00 7.215377e+02 1.728540e+02 2.163791e-01 0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03";
    std::string strExpected = "721.5377 0.0 609.5593 44.85728 0.0 721.5377 172.854 0.2163791 0.0 0.0 1.0 0.002745884 0.0 0.0 0.0 1.0";

    auto P_rect_02 = KittiReader::splitLineByChar(strP_rect_02, ' ');
    auto vecExpected = KittiReader::splitLineByChar(strExpected, ' ');

    auto expected = cv::Mat(vecExpected).reshape(1, 4);
    auto actual = Calibration::getCam0RectToImage2(P_rect_02);
    REQUIRE(areEqual(expected, actual));
}

TEST_CASE( "cam0UnrectToCam2Rect", "[xforms]" )
{
    std::string strR_rect_00 = "9.999239e-01 9.837760e-03 -7.445048e-03 -9.869795e-03 9.999421e-01 -4.278459e-03 7.402527e-03 4.351614e-03 9.999631e-01";
    std::string strP_rect_02 = "7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01 0.000000e+00 7.215377e+02 1.728540e+02 2.163791e-01 0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03";
    std::string strExpected = "0.9999239 0.00983776 -0.007445048 0.06216900378178438 -0.009869795 0.9999421 -0.004278459 0.0 0.007402527 0.004351614 0.9999631 0.0 0.0 0.0 0.0 1.0";

    auto R_rect_00 = KittiReader::splitLineByChar(strR_rect_00, ' ');
    auto P_rect_02 = KittiReader::splitLineByChar(strP_rect_02, ' ');
    auto vecExpected = KittiReader::splitLineByChar(strExpected, ' ');

    auto expected = cv::Mat(vecExpected).reshape(1, 4);
    auto actual = Calibration::getCam0UnrectToCam2Rect(R_rect_00, P_rect_02);
    REQUIRE(areEqual(expected, actual));
}