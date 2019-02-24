#include "catch.hpp"
#include "makerosbag.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

TEST_CASE( "Read P matrix", "[P]" )
{
    std::string strR = "0.007533745 -0.9999714 -0.000616602 0.01480249 \
        0.0007280733 -0.9998902 0.9998621 0.00752379 0.01480755";
    std::string strT = "-0.004069766 -0.07631618 -0.2717806";

    auto R = Calibration::splitLineByChar(strR, ' ');
    auto T = Calibration::splitLineByChar(strT, ' ');

    REQUIRE(1 == 1);
}

TEST_CASE( "Read R matrix", "[R]" )
{
    REQUIRE(1 == 1);
}
