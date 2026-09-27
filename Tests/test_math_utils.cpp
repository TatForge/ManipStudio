#include "gtest/gtest.h"
#include "my_math.h"

#include <thread>
#include <chrono>

using namespace RoboMath;

TEST(MathUtils, TraceOfMatrix4x4)
{
    Matrix4x4 m({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    EXPECT_DOUBLE_EQ(Trace(m), 1 + 6 + 11 + 16);
}

TEST(MathUtils, TraceOfMatrix)
{
    Matrix m{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    EXPECT_DOUBLE_EQ(Trace(m), 1 + 5 + 9);
}

TEST(MathUtils, DistanceBetweenStateVectorsIgnoresOrientation)
{
    StateVector a(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    StateVector b(3.0, 4.0, 0.0, 1.0, 2.0, 3.0); // large orientation diff, should not matter
    EXPECT_NEAR(Distance(a, b), 5.0, 1e-9);
}

TEST(MathUtils, CalcDistancePointToLine)
{
    Vector3 l1(0, 0, 0);
    Vector3 l2(1, 0, 0);
    Vector3 p(0, 1, 0);
    EXPECT_NEAR(CalcDistancePointToLine(p, &l1, &l2), 1.0, 1e-9);
}

TEST(MathUtils, CalcDistancePointToLineZeroWhenOnLine)
{
    Vector3 l1(0, 0, 0);
    Vector3 l2(2, 0, 0);
    Vector3 p(1, 0, 0);
    EXPECT_NEAR(CalcDistancePointToLine(p, &l1, &l2), 0.0, 1e-9);
}

TEST(MathUtils, MinMax)
{
    EXPECT_EQ(RoboMath::min(3, 7), 3);
    EXPECT_EQ(RoboMath::max(3, 7), 7);
    EXPECT_DOUBLE_EQ(RoboMath::min(2.5, -1.0), -1.0);
    EXPECT_DOUBLE_EQ(RoboMath::max(2.5, -1.0), 2.5);
}

TEST(MathUtils, Interpolate)
{
    EXPECT_DOUBLE_EQ(RoboMath::interpolate(0.0, 10.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(RoboMath::interpolate(0.0, 10.0, 1.0), 10.0);
    EXPECT_DOUBLE_EQ(RoboMath::interpolate(0.0, 10.0, 0.5), 5.0);
}

TEST(MathUtils, SignSameSign)
{
    EXPECT_TRUE(RoboMath::sign(2.0, 3.0));
    EXPECT_TRUE(RoboMath::sign(-2.0, -3.0));
}

TEST(MathUtils, SignOppositeSign)
{
    EXPECT_FALSE(RoboMath::sign(2.0, -3.0));
    EXPECT_FALSE(RoboMath::sign(-2.0, 3.0));
}

TEST(CheckTime, DurationIsNonNegative)
{
    CheckTime ct;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    auto d = ct.duration();
    EXPECT_GE(d.count(), 0.0);
}

TEST(CheckTime, DurationIsMonotonicNonDecreasing)
{
    CheckTime ct;
    auto first = ct.duration();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    auto second = ct.duration();
    EXPECT_GE(second.count(), first.count());
}
