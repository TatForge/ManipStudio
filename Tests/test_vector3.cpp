#include "gtest/gtest.h"
#include "my_math.h"

using namespace RoboMath;

TEST(Vector3, DefaultConstructorIsZero)
{
    Vector3 v;
    EXPECT_DOUBLE_EQ(v.x, 0);
    EXPECT_DOUBLE_EQ(v.y, 0);
    EXPECT_DOUBLE_EQ(v.z, 0);
}

TEST(Vector3, ComponentConstructor)
{
    Vector3 v(1, 2, 3);
    EXPECT_DOUBLE_EQ(v.x, 1);
    EXPECT_DOUBLE_EQ(v.y, 2);
    EXPECT_DOUBLE_EQ(v.z, 3);
}

TEST(Vector3, Length)
{
    Vector3 v(3, 4, 0);
    EXPECT_DOUBLE_EQ(v.Length(), 5.0);
}

TEST(Vector3, NormalizeUnitLength)
{
    Vector3 v(3, 4, 0);
    Vector3 n = v.Normalize();
    EXPECT_NEAR(n.Length(), 1.0, 1e-9);
    EXPECT_NEAR(n.x, 0.6, 1e-9);
    EXPECT_NEAR(n.y, 0.8, 1e-9);
}

TEST(Vector3, NormalizeZeroVectorReturnsZero)
{
    Vector3 v(0, 0, 0);
    Vector3 n = v.Normalize();
    EXPECT_DOUBLE_EQ(n.x, 0);
    EXPECT_DOUBLE_EQ(n.y, 0);
    EXPECT_DOUBLE_EQ(n.z, 0);
}

TEST(Vector3, Dot)
{
    Vector3 a(1, 2, 3);
    Vector3 b(4, -5, 6);
    EXPECT_DOUBLE_EQ(a.Dot(b), 1 * 4 + 2 * -5 + 3 * 6);
}

TEST(Vector3, CrossProduct)
{
    Vector3 x(1, 0, 0);
    Vector3 y(0, 1, 0);
    Vector3 z = x * y;
    EXPECT_NEAR(z.x, 0, 1e-9);
    EXPECT_NEAR(z.y, 0, 1e-9);
    EXPECT_NEAR(z.z, 1, 1e-9);
}

TEST(Vector3, TriangleSIsTwiceTriangleArea)
{
    // Right triangle with legs 3 and 4 -> area 6, cross-product magnitude 12
    Vector3 a(0, 0, 0);
    Vector3 b(3, 0, 0);
    Vector3 leg = Vector3(0, 4, 0) - a;
    scalar s = b.TriangleS(leg);
    EXPECT_NEAR(s, 12.0, 1e-9);
}

TEST(Vector3, AngleBetweenPerpendicularVectors)
{
    Vector3 a(1, 0, 0);
    Vector3 b(0, 1, 0);
    EXPECT_NEAR(a.Angle(b), M_PI / 2, 1e-9);
}

TEST(Vector3, AngleBetweenParallelVectors)
{
    Vector3 a(2, 0, 0);
    Vector3 b(5, 0, 0);
    EXPECT_NEAR(a.Angle(b), 0.0, 1e-6);
}

TEST(Vector3, RotateAroundZByNinetyDegrees)
{
    Vector3 v(1, 0, 0);
    v.Rotate(M_PI / 2, unit_z);
    EXPECT_NEAR(v.x, 0.0, 1e-9);
    EXPECT_NEAR(v.y, 1.0, 1e-9);
    EXPECT_NEAR(v.z, 0.0, 1e-9);
}

TEST(Vector3, OperatorPlus)
{
    Vector3 a(1, 2, 3);
    Vector3 b(4, 5, 6);
    Vector3 c = a + b;
    EXPECT_DOUBLE_EQ(c.x, 5);
    EXPECT_DOUBLE_EQ(c.y, 7);
    EXPECT_DOUBLE_EQ(c.z, 9);
}

TEST(Vector3, OperatorMinus)
{
    Vector3 a(5, 7, 9);
    Vector3 b(1, 2, 3);
    Vector3 c = a - b;
    EXPECT_DOUBLE_EQ(c.x, 4);
    EXPECT_DOUBLE_EQ(c.y, 5);
    EXPECT_DOUBLE_EQ(c.z, 6);
}

TEST(Vector3, UnaryMinus)
{
    Vector3 a(1, -2, 3);
    Vector3 b = -a;
    EXPECT_DOUBLE_EQ(b.x, -1);
    EXPECT_DOUBLE_EQ(b.y, 2);
    EXPECT_DOUBLE_EQ(b.z, -3);
}

TEST(Vector3, OperatorPlusEquals)
{
    Vector3 a(1, 2, 3);
    a += Vector3(1, 1, 1);
    EXPECT_DOUBLE_EQ(a.x, 2);
    EXPECT_DOUBLE_EQ(a.y, 3);
    EXPECT_DOUBLE_EQ(a.z, 4);
}

// Regression test for a fixed bug: operator-= is copy-pasted from operator+= and adds
// instead of subtracting (my_math.cpp, Vector3::operator-=). This test
// expects correct subtraction semantics.
TEST(Vector3, OperatorMinusEqualsSubtracts)
{
    Vector3 a(5, 5, 5);
    a -= Vector3(1, 2, 3);
    EXPECT_DOUBLE_EQ(a.x, 4);
    EXPECT_DOUBLE_EQ(a.y, 3);
    EXPECT_DOUBLE_EQ(a.z, 2);
}

TEST(Vector3, ScalarMultiply)
{
    Vector3 a(1, 2, 3);
    Vector3 b = a * 2.0;
    EXPECT_DOUBLE_EQ(b.x, 2);
    EXPECT_DOUBLE_EQ(b.y, 4);
    EXPECT_DOUBLE_EQ(b.z, 6);
}

TEST(Vector3, ScalarDivide)
{
    Vector3 a(2, 4, 6);
    Vector3 b = a / 2.0;
    EXPECT_DOUBLE_EQ(b.x, 1);
    EXPECT_DOUBLE_EQ(b.y, 2);
    EXPECT_DOUBLE_EQ(b.z, 3);
}

TEST(Vector3, InequalityOperator)
{
    Vector3 a(1, 2, 3);
    Vector3 b(1, 2, 3);
    Vector3 c(1, 2, 4);
    EXPECT_FALSE(a != b);
    EXPECT_TRUE(a != c);
}

TEST(Vector3, LessThanOperatorLexicographic)
{
    Vector3 a(1, 2, 3);
    Vector3 b(2, 0, 0);
    Vector3 c(1, 3, 0);
    Vector3 d(1, 2, 4);
    EXPECT_TRUE(a < b);
    EXPECT_TRUE(a < c);
    EXPECT_TRUE(a < d);
    EXPECT_FALSE(b < a);
}

TEST(Vector3, Clear)
{
    Vector3 a(1, 2, 3);
    a.Clear();
    EXPECT_DOUBLE_EQ(a.x, 0);
    EXPECT_DOUBLE_EQ(a.y, 0);
    EXPECT_DOUBLE_EQ(a.z, 0);
}

TEST(Vector3, AssignFromVector)
{
    Vector v({1, 2, 3});
    Vector3 a;
    a = v;
    EXPECT_DOUBLE_EQ(a.x, 1);
    EXPECT_DOUBLE_EQ(a.y, 2);
    EXPECT_DOUBLE_EQ(a.z, 3);
}
