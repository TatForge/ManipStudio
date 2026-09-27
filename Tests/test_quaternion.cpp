#include "gtest/gtest.h"
#include "my_math.h"

using namespace RoboMath;

// Note: Quaternion's x/y/z/w components are private with no accessor, so
// these tests only observe behaviour through the public API (GetAngle,
// GetAxis, GetLength, GetEulerAngles, RotateVector).

TEST(Quaternion, DefaultConstructorIsIdentity)
{
    Quaternion q;
    EXPECT_NEAR(q.GetAngle(), 0.0, 1e-5);
}

TEST(Quaternion, AxisAngleConstructorRoundTripsAngle)
{
    Quaternion q(unit_z, M_PI / 2);
    EXPECT_NEAR(static_cast<double>(q.GetAngle()), M_PI / 2, 1e-9);
}

TEST(Quaternion, AxisAngleConstructorRoundTripsAxis)
{
    Quaternion q(unit_z, M_PI / 2);
    Vector3 axis = q.GetAxis();
    EXPECT_NEAR(axis.x, 0.0, 1e-9);
    EXPECT_NEAR(axis.y, 0.0, 1e-9);
    EXPECT_NEAR(axis.z, 1.0, 1e-9);
}

TEST(Quaternion, CreateMatchesConstructor)
{
    Quaternion q;
    q.Create(unit_x, M_PI / 3);
    EXPECT_NEAR(static_cast<double>(q.GetAngle()), M_PI / 3, 1e-9);
    Vector3 axis = q.GetAxis();
    EXPECT_NEAR(axis.x, 1.0, 1e-9);
}

TEST(Quaternion, RotateVectorAroundZByNinetyDegrees)
{
    Quaternion q(unit_z, M_PI / 2);
    Vector3 v = q.RotateVector(Vector3(1, 0, 0));
    EXPECT_NEAR(v.x, 0.0, 1e-9);
    EXPECT_NEAR(v.y, 1.0, 1e-9);
    EXPECT_NEAR(v.z, 0.0, 1e-9);
}

TEST(Quaternion, InvertReversesRotationRoundTrip)
{
    Quaternion q(unit_z, M_PI / 3);
    Vector3 original(1, 0, 0);
    Vector3 rotated = q.RotateVector(original);
    q.Invert();
    Vector3 back = q.RotateVector(rotated);
    EXPECT_NEAR(back.x, original.x, 1e-9);
    EXPECT_NEAR(back.y, original.y, 1e-9);
    EXPECT_NEAR(back.z, original.z, 1e-9);
}

TEST(Quaternion, EulerToQuatAndBackRoundTrip)
{
    Vector3 angles_in(0.3, 0.2, 0.4); // roll, pitch, yaw (radians), away from gimbal lock
    Quaternion q;
    q.EulerToQuat(angles_in);
    Vector3 angles_out = q.GetEulerAngles();
    EXPECT_NEAR(angles_out.x, angles_in.x, 1e-6);
    EXPECT_NEAR(angles_out.y, angles_in.y, 1e-6);
    EXPECT_NEAR(angles_out.z, angles_in.z, 1e-6);
}

// GetLength() returns the squared norm (x^2+y^2+z^2+w^2), not sqrt of it --
// this test documents the actual behaviour rather than asserting the more
// intuitive sqrt-based length.
TEST(Quaternion, GetLengthReturnsSquaredNorm)
{
    Quaternion q(1, 2, 3, 4); // x, y, z, w
    EXPECT_NEAR(q.GetLength(), 1 + 4 + 9 + 16, 1e-9);
}

TEST(Quaternion, NormalizeProducesUnitSquaredNorm)
{
    Quaternion q(2, 4, 6, 8);
    q.Normalize();
    EXPECT_NEAR(q.GetLength(), 1.0, 1e-9);
}

TEST(Quaternion, ScaleScalesSquaredNormBySquareOfFactor)
{
    Quaternion q(1, 2, 3, 4);
    q.Scale(2);
    EXPECT_NEAR(q.GetLength(), 30.0 * 4, 1e-9);
}

TEST(Quaternion, MultiplyByIdentityIsUnchanged)
{
    Quaternion q(unit_z, M_PI / 2);
    Quaternion identity;
    Quaternion result = identity * q;
    EXPECT_NEAR(static_cast<double>(result.GetAngle()), static_cast<double>(q.GetAngle()), 1e-6);
}

TEST(Quaternion, MultiplyByVector3IdentityCase)
{
    Quaternion identity;
    Quaternion result = identity * Vector3(1, 2, 3);
    // xx=1, yy=2, zz=3, ww=0 for the identity quaternion (see my_math.cpp)
    EXPECT_NEAR(result.GetLength(), 1 + 4 + 9 + 0, 1e-9);
}
