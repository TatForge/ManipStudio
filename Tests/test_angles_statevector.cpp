#include "gtest/gtest.h"
#include "my_math.h"

using namespace RoboMath;

TEST(Angles, DefaultConstructorIsZero)
{
    Angles a;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_DOUBLE_EQ(a.q[i], 0);
        EXPECT_DOUBLE_EQ(a.q_p[i], 0);
        EXPECT_DOUBLE_EQ(a.q_pp[i], 0);
    }
}

TEST(Angles, IsNullTrueForDefault)
{
    Angles a;
    EXPECT_TRUE(a.IsNull());
}

TEST(Angles, IsNullFalseWhenAnyFieldSet)
{
    Angles a;
    a.q[2] = 0.5;
    EXPECT_FALSE(a.IsNull());

    Angles b;
    b.q_p[0] = 0.1;
    EXPECT_FALSE(b.IsNull());

    Angles c;
    c.q_pp[5] = 0.1;
    EXPECT_FALSE(c.IsNull());
}

TEST(Angles, EqualityComparesPositionOnly)
{
    Angles a;
    Angles b;
    for (int i = 0; i < JOINTS_COUNT; i++)
        a.q[i] = b.q[i] = i * 0.1;

    EXPECT_TRUE(a == b);

    // operator== in my_math.h only compares q[], not q_p/q_pp -- this test
    // documents that (intentional or not) actual behaviour.
    b.q_p[0] = 99;
    b.q_pp[0] = 99;
    EXPECT_TRUE(a == b);

    b.q[0] = 99;
    EXPECT_FALSE(a == b);
}

TEST(StateVector, DefaultConstructorIsZero)
{
    StateVector v;
    EXPECT_DOUBLE_EQ(v.x, 0);
    EXPECT_DOUBLE_EQ(v.y, 0);
    EXPECT_DOUBLE_EQ(v.z, 0);
    EXPECT_DOUBLE_EQ(v.roll, 0);
    EXPECT_DOUBLE_EQ(v.pitch, 0);
    EXPECT_DOUBLE_EQ(v.yaw, 0);
}

TEST(StateVector, ComponentConstructor)
{
    StateVector v(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 2.0);
    EXPECT_DOUBLE_EQ(v.z, 3.0);
    EXPECT_DOUBLE_EQ(v.roll, 0.1);
    EXPECT_DOUBLE_EQ(v.pitch, 0.2);
    EXPECT_DOUBLE_EQ(v.yaw, 0.3);
}

TEST(StateVector, ConstructFromCoordAndAngleVectors)
{
    Vector3 r(1, 2, 3);
    Vector3 a(0.1, 0.2, 0.3);
    StateVector v(r, a);
    EXPECT_DOUBLE_EQ(v.x, 1);
    EXPECT_DOUBLE_EQ(v.y, 2);
    EXPECT_DOUBLE_EQ(v.z, 3);
    EXPECT_DOUBLE_EQ(v.roll, 0.1);
    EXPECT_DOUBLE_EQ(v.pitch, 0.2);
    EXPECT_DOUBLE_EQ(v.yaw, 0.3);
}

TEST(StateVector, GetCoord)
{
    StateVector v(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    Vector3 coord = v.GetCoord();
    EXPECT_DOUBLE_EQ(coord.x, 1);
    EXPECT_DOUBLE_EQ(coord.y, 2);
    EXPECT_DOUBLE_EQ(coord.z, 3);
}

TEST(StateVector, GetAngles)
{
    StateVector v(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    Vector3 angles = v.GetAngles();
    EXPECT_DOUBLE_EQ(angles.x, 0.1);
    EXPECT_DOUBLE_EQ(angles.y, 0.2);
    EXPECT_DOUBLE_EQ(angles.z, 0.3);
}
