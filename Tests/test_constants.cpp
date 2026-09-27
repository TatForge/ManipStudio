#include "gtest/gtest.h"
#include "constants.h"

#include <cmath>

TEST(Constants, ToRadConvertsDegrees)
{
    EXPECT_NEAR(ToRad(180.0), M_PI, 1e-12);
    EXPECT_NEAR(ToRad(90.0), M_PI / 2, 1e-12);
    EXPECT_NEAR(ToRad(-45.0), -M_PI / 4, 1e-12);
    EXPECT_DOUBLE_EQ(ToRad(0.0), 0.0);
}

TEST(Constants, ToDegConvertsRadians)
{
    EXPECT_NEAR(ToDeg(M_PI), 180.0, 1e-12);
    EXPECT_NEAR(ToDeg(-M_PI / 2), -90.0, 1e-12);
}

TEST(Constants, ToRadToDegRoundTrip)
{
    for (double deg = -720; deg <= 720; deg += 17.5)
        EXPECT_NEAR(ToDeg(ToRad(deg)), deg, 1e-9) << "deg = " << deg;
}

TEST(Constants, ToSecConvertsMilliseconds)
{
    EXPECT_DOUBLE_EQ(ToSec(1000), 1.0);
    EXPECT_DOUBLE_EQ(ToSec(10), 0.01);
    EXPECT_DOUBLE_EQ(ToSec(1500), 1.5);
}

TEST(Constants, ToMillisecRoundsToNearestMillisecond)
{
    EXPECT_DOUBLE_EQ(ToMillisec(1.0), 1000);
    EXPECT_DOUBLE_EQ(ToMillisec(0.0014), 1);
    EXPECT_DOUBLE_EQ(ToMillisec(0.0016), 2);
    EXPECT_DOUBLE_EQ(ToMillisec(1.2345), 1235);
}

TEST(Constants, ToSecToMillisecRoundTrip)
{
    for (int ms = 0; ms <= 5000; ms += 7)
        EXPECT_DOUBLE_EQ(ToMillisec(ToSec(ms)), ms) << "ms = " << ms;
}

TEST(Constants, PhysicalAndSolverConstantsAreSane)
{
    EXPECT_EQ(JOINTS_COUNT, 6);
    EXPECT_NEAR(g, 9.80665, 1e-12);
    EXPECT_DOUBLE_EQ(damping_constant_sq, damping_constant * damping_constant);
    EXPECT_NEAR(MaxAngleDls, M_PI / 4, 1e-12);
    EXPECT_GT(MAX_KINEMATIC_ITERS, 0);
    EXPECT_GT(ACCURACY, 0);
    EXPECT_GT(DELTA_RANGE, 0);
}
