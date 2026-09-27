#include "gtest/gtest.h"
#include "my_math.h"

#include <algorithm>
#include <cmath>

using namespace RoboMath;

TEST(Vector, InitializerListConstructor)
{
    Vector v({1, 2, 3, 4});
    EXPECT_EQ(v.GetLength(), 4);
    EXPECT_DOUBLE_EQ(v.at(0), 1);
    EXPECT_DOUBLE_EQ(v.at(3), 4);
}

TEST(Vector, SizeValueConstructor)
{
    Vector v(5, 2.5);
    EXPECT_EQ(v.GetLength(), 5);
    for (int i = 0; i < 5; i++)
        EXPECT_DOUBLE_EQ(v.at(i), 2.5);
}

TEST(Vector, CopyConstructorIsDeepCopy)
{
    Vector a({1, 2, 3});
    Vector b(a);
    b(0) = 99;
    EXPECT_DOUBLE_EQ(a.at(0), 1);
    EXPECT_DOUBLE_EQ(b.at(0), 99);
}

TEST(Vector, OperatorParenReadWrite)
{
    Vector v({1, 2, 3});
    v(1) = 42;
    EXPECT_DOUBLE_EQ(v(1), 42);
}

TEST(Vector, MultiplyTranspose)
{
    EXPECT_EQ(Vector({1, 2, 3, 4}).MultiplyTranspose(Vector{3, 4, 5, 6}), 50);
    EXPECT_EQ(Vector({1, 2, 3, 4}).MultiplyTranspose(Vector{0, 0, 0, 0}), 0);
}

TEST(Vector, EqualityOperator)
{
    EXPECT_TRUE(Vector({1, 2, 3, 4}) == Vector({1, 2, 3, 4}));
    EXPECT_TRUE(Vector({-1.22, -5.009, 123, 54.0009}) == Vector({-1.22, -5.009, 123, 54.0009}));
    EXPECT_FALSE(Vector({1, 2, 3, 4}) == Vector({-1, 2, 3, 4}));
    EXPECT_FALSE(Vector({1, 2, 3, 4}) == Vector({1, 2.001, 3.001, 4}));
}

TEST(Vector, OperatorTimesEqualsScalar)
{
    Vector v({1, 2, 3});
    v *= 2;
    EXPECT_TRUE(v == Vector({2, 4, 6}));
}

TEST(Vector, AssignFromDoubleFillsAllElements)
{
    Vector v({1, 2, 3});
    v = 7.0;
    EXPECT_TRUE(v == Vector({7, 7, 7}));
}

TEST(Vector, CopyAssignmentIsDeepCopy)
{
    Vector a({1, 2, 3});
    Vector b({0, 0, 0});
    b = a;
    b(0) = 99;
    EXPECT_DOUBLE_EQ(a.at(0), 1);
}

TEST(Vector, MaxAbs)
{
    Vector v({-1, 5, -8, 3});
    EXPECT_DOUBLE_EQ(v.MaxAbs(), 8);
}

// Regression test for a fixed bug: operator-(Vector) in my_math.cpp hardcodes indices
// 0..2 (ignoring the actual vector length) and computes v - this instead of
// this - v. This test expects correct, length-independent left-minus-right
// semantics.
TEST(Vector, OperatorMinusComputesLeftMinusRight)
{
    Vector a({10, 20, 30});
    Vector b({1, 2, 3});
    Vector result = a - b;
    EXPECT_TRUE(result == Vector({9, 18, 27}));
}

// NOTE: Vector::operator*(const Vector&) is declared in my_math.h but has no
// definition in my_math.cpp -- calling it would be a link error. Not tested
// here; see the audit notes for this newly-found issue.

TEST(Vector, DefaultConstructorIsEmpty)
{
    Vector v;
    EXPECT_EQ(v.GetLength(), 0);
    EXPECT_EQ(v.GetDataPtr(), nullptr);
}

TEST(Vector, SizedConstructorHasRequestedLength)
{
    Vector v(7);
    EXPECT_EQ(v.GetLength(), 7);
    EXPECT_NE(v.GetDataPtr(), nullptr);
}

TEST(Vector, ConstAtReadsElements)
{
    const Vector v({1.5, -2.5, 3.5});
    EXPECT_DOUBLE_EQ(v.at(0), 1.5);
    EXPECT_DOUBLE_EQ(v.at(2), 3.5);
}

TEST(Vector, SelfAssignmentKeepsData)
{
    Vector v({1, 2, 3});
    Vector &ref = v;
    v = ref;
    EXPECT_EQ(v.GetLength(), 3);
    EXPECT_DOUBLE_EQ(v.at(1), 2);
}

// Vector::operator- hardcodes three components, so for any other length the
// remaining elements are left uninitialised. (The sign is covered by
// OperatorMinusComputesLeftMinusRight.)
TEST(Vector, OperatorMinusHandlesAllElements)
{
    Vector a({5, 5, 5, 5, 5, 5});
    Vector b({1, 2, 3, 4, 5, 6});
    Vector r = a - b;
    ASSERT_EQ(r.GetLength(), 6);
    for (int i = 3; i < 6; i++)
        EXPECT_NEAR(std::abs(r.at(i)), std::abs(5.0 - (i + 1)), 1e-12) << "i = " << i;
}

TEST(Vector, MultiplyTransposeOfLongVectors)
{
    Vector a({1, 2, 3, 4, 5, 6});
    Vector b({6, 5, 4, 3, 2, 1});
    EXPECT_DOUBLE_EQ(a.MultiplyTranspose(b), 56);
}
