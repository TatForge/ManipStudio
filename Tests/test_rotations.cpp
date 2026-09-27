// Cross-checks between the different rotation representations in my_math
// (Quaternion, Matrix4x4, Euler angles, Vector3::Rotate). These are the
// building blocks of forward/inverse kinematics, so every representation must
// describe the *same* physical rotation.

#include "gtest/gtest.h"
#include "my_math.h"
#include "test_helpers.h"

#include <cmath>
#include <vector>

using namespace RoboMath;
using namespace test_helpers;

namespace {

Matrix4x4 ToMatrix4x4(const Mat4 &m)
{
    Matrix4x4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r(i, j) = m[i][j];
    return r;
}

Vector3 Apply(const Mat4 &m, const Vector3 &v)
{
    return Vector3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                   m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                   m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
}

// Rotation matrix for rotation by `angle` around unit `axis` (Rodrigues).
Mat4 AxisAngleMatrix(Vector3 axis, double angle)
{
    axis = axis.Normalize();
    double c = cos(angle), s = sin(angle), t = 1 - c;
    double x = axis.x, y = axis.y, z = axis.z;
    Mat4 m = Identity4();
    m[0][0] = t * x * x + c;     m[0][1] = t * x * y - s * z; m[0][2] = t * x * z + s * y;
    m[1][0] = t * x * y + s * z; m[1][1] = t * y * y + c;     m[1][2] = t * y * z - s * x;
    m[2][0] = t * x * z - s * y; m[2][1] = t * y * z + s * x; m[2][2] = t * z * z + c;
    return m;
}

// ZYX (yaw-pitch-roll) convention used by Quaternion::EulerToQuat /
// GetEulerAngles: R = Rz(yaw) * Ry(pitch) * Rx(roll).
Mat4 EulerMatrix(double roll, double pitch, double yaw)
{
    Mat4 ry = Identity4();
    ry[0][0] = cos(pitch);  ry[0][2] = sin(pitch);
    ry[2][0] = -sin(pitch); ry[2][2] = cos(pitch);
    return Mul(Mul(RotZ(yaw), ry), RotX(roll));
}

const std::vector<Vector3> &ProbeVectors()
{
    static const std::vector<Vector3> v{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0.3, -0.7, 0.2}, {-1.5, 2.0, 0.5}};
    return v;
}

const std::vector<Vector3> &Axes()
{
    static const std::vector<Vector3> v{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {1, 1, 1}, {-0.3, 0.8, 0.5}, {0.2, -0.1, -0.9}};
    return v;
}

const std::vector<double> &AnglesGrid()
{
    static const std::vector<double> v{-3.0, -2.0, -M_PI / 2, -0.7, -0.1, 0.05, 0.4, 1.0, M_PI / 2, 2.5, 3.1};
    return v;
}

} // namespace

TEST(Rotations, QuaternionRotateVectorMatchesRodriguesMatrix)
{
    for (const auto &axis : Axes())
        for (double angle : AnglesGrid())
        {
            Quaternion q(axis, angle);
            Mat4 m = AxisAngleMatrix(axis, angle);
            for (const auto &v : ProbeVectors())
                EXPECT_VEC3_NEAR(q.RotateVector(v), Apply(m, v), 1e-9);
        }
}

TEST(Rotations, Vector3RotateMatchesQuaternionRotation)
{
    for (const auto &axis : Axes())
        for (double angle : AnglesGrid())
            for (const auto &v : ProbeVectors())
            {
                Vector3 rotated = v;
                rotated.Rotate(angle, axis.Normalize());
                EXPECT_VEC3_NEAR(rotated, Quaternion(axis, angle).RotateVector(v), 1e-9);
            }
}

TEST(Rotations, RotationPreservesLength)
{
    for (const auto &axis : Axes())
        for (double angle : AnglesGrid())
            for (const auto &v : ProbeVectors())
                EXPECT_NEAR(Quaternion(axis, angle).RotateVector(v).Length(), v.Length(), 1e-9);
}

TEST(Rotations, GetAxisAndGetAngleRecoverAxisAngle)
{
    for (const auto &axis : Axes())
        for (double angle : {0.1, 0.9, 1.7, 2.9})
        {
            Quaternion q(axis, angle);
            EXPECT_NEAR(static_cast<double>(q.GetAngle()), angle, 1e-9);
            EXPECT_VEC3_NEAR(q.GetAxis(), axis.Normalize(), 1e-9);
        }
}

TEST(Rotations, QuaternionProductComposesRotations)
{
    Quaternion q1(Vector3(0.2, 0.9, -0.4), 1.1);
    Quaternion q2(Vector3(-0.5, 0.1, 0.8), -2.3);
    Quaternion q12 = q1 * q2; // apply q2 first, then q1
    for (const auto &v : ProbeVectors())
        EXPECT_VEC3_NEAR(q12.RotateVector(v), q1.RotateVector(q2.RotateVector(v)), 1e-9);
}

TEST(Rotations, InverseQuaternionUndoesRotation)
{
    for (const auto &axis : Axes())
        for (double angle : AnglesGrid())
        {
            Quaternion q(axis, angle);
            Quaternion inv = q;
            inv.Invert();
            for (const auto &v : ProbeVectors())
                EXPECT_VEC3_NEAR(inv.RotateVector(q.RotateVector(v)), v, 1e-9);
        }
}

TEST(Rotations, EulerToQuatMatchesZYXMatrix)
{
    for (double roll : {-2.5, -0.3, 0.0, 0.8, 3.0})
        for (double pitch : {-1.2, -0.4, 0.0, 0.6, 1.3})
            for (double yaw : {-3.0, -1.0, 0.0, 0.5, 2.2})
            {
                Quaternion q;
                q.EulerToQuat(Vector3(roll, pitch, yaw));
                Mat4 m = EulerMatrix(roll, pitch, yaw);
                for (const auto &v : ProbeVectors())
                    EXPECT_VEC3_NEAR(q.RotateVector(v), Apply(m, v), 1e-9)
                        << "roll " << roll << " pitch " << pitch << " yaw " << yaw;
            }
}

TEST(Rotations, EulerRoundTripAwayFromGimbalLock)
{
    for (double roll : {-2.5, -0.3, 0.0, 0.8, 3.0})
        for (double pitch : {-1.2, -0.4, 0.0, 0.6, 1.3})
            for (double yaw : {-3.0, -1.0, 0.0, 0.5, 2.2})
            {
                Quaternion q;
                q.EulerToQuat(Vector3(roll, pitch, yaw));
                Vector3 back = q.GetEulerAngles();
                EXPECT_NEAR(AngleDiff(back.x, roll), 0, 1e-9);
                EXPECT_NEAR(AngleDiff(back.y, pitch), 0, 1e-9);
                EXPECT_NEAR(AngleDiff(back.z, yaw), 0, 1e-9);
            }
}

// At pitch = +-90 deg roll and yaw are not unique, but the recovered angles
// must still describe the same rotation.
TEST(Rotations, EulerAtGimbalLockStillDescribesSameRotation)
{
    for (double pitch : {M_PI / 2, -M_PI / 2})
        for (double roll : {-1.0, 0.3})
            for (double yaw : {0.0, 0.7})
            {
                Quaternion q;
                q.EulerToQuat(Vector3(roll, pitch, yaw));
                Vector3 back = q.GetEulerAngles();
                ASSERT_TRUE(std::isfinite(back.x) && std::isfinite(back.y) && std::isfinite(back.z));
                Quaternion q2;
                q2.EulerToQuat(back);
                for (const auto &v : ProbeVectors())
                    EXPECT_VEC3_NEAR(q2.RotateVector(v), q.RotateVector(v), 1e-6);
            }
}

TEST(Rotations, Matrix4x4ToQuaternionForGeneralRotations)
{
    for (const auto &axis : Axes())
        for (double angle : AnglesGrid())
        {
            Mat4 m = AxisAngleMatrix(axis, angle);
            Quaternion q = ToMatrix4x4(m).ConvertToQuaternion();
            for (const auto &v : ProbeVectors())
                EXPECT_VEC3_NEAR(q.RotateVector(v), Apply(m, v), 1e-9)
                    << "axis (" << axis.x << "," << axis.y << "," << axis.z << ") angle " << angle;
        }
}

// Rotations by exactly 180 deg make trace(R) = -1, which sends
// ConvertToQuaternion into its `else` branch. That branch picks the case by
// Matrix4x4::MaxElem() (largest element of the whole 3x3 block instead of the
// largest *diagonal* element) and assembles the quaternion from wrong
// components (e.g. 0.5/s instead of 0.25*s, and (a6+a9) instead of (a9-a6)
// for w). Rotations about the coordinate axes happen to survive this, but
// any other axis produces a wrong rotation.
TEST(Rotations, Matrix4x4ToQuaternionForHalfTurns)
{
    for (const auto &axis : Axes())
    {
        Mat4 m = AxisAngleMatrix(axis, M_PI);
        Quaternion q = ToMatrix4x4(m).ConvertToQuaternion();
        for (const auto &v : ProbeVectors())
            EXPECT_VEC3_NEAR(q.RotateVector(v), Apply(m, v), 1e-9)
                << "axis (" << axis.x << "," << axis.y << "," << axis.z << ")";
    }
}

// Regression net for the half-turn fix: rotations *close to* 180 deg go
// through the trace > 0 branch today and must keep working, on both sides
// of 180 deg and for the axes that matter to the robot.
TEST(Rotations, Matrix4x4ToQuaternionNearHalfTurns)
{
    for (const auto &axis : Axes())
        for (double eps : {-1e-2, -1e-4, -1e-6, 1e-6, 1e-4, 1e-2})
        {
            Mat4 m = AxisAngleMatrix(axis, M_PI + eps);
            Quaternion q = ToMatrix4x4(m).ConvertToQuaternion();
            for (const auto &v : ProbeVectors())
                EXPECT_VEC3_NEAR(q.RotateVector(v), Apply(m, v), 1e-8)
                    << "axis (" << axis.x << "," << axis.y << "," << axis.z << ") eps " << eps;
        }
}

// "Tool pointing straight down" (tool z -> world -Z) with ANY yaw is a
// symmetric rotation matrix, i.e. an exact half turn about a horizontal
// axis -- so every such pose (the zero / internal pose, typical
// pick-and-place orientations) goes through the defective trace <= 0 branch.
TEST(Rotations, Matrix4x4ToQuaternionForToolPointingDown)
{
    for (double yaw : {0.0, 1e-9, -1e-7, 0.3})
    {
        Mat4 m = Mul(RotZ(yaw), AxisAngleMatrix(Vector3(1, -1, 0), M_PI));
        Quaternion q = ToMatrix4x4(m).ConvertToQuaternion();
        for (const auto &v : ProbeVectors())
            EXPECT_VEC3_NEAR(q.RotateVector(v), Apply(m, v), 1e-8) << "yaw " << yaw;
    }
}

TEST(Rotations, Matrix4x4ProductMatchesComposedQuaternions)
{
    Mat4 a = AxisAngleMatrix(Vector3(1, 2, 3), 0.7);
    Mat4 b = AxisAngleMatrix(Vector3(-1, 0.5, 0), -1.9);
    Matrix4x4 ab = ToMatrix4x4(a) * ToMatrix4x4(b);
    Quaternion qab = ToMatrix4x4(a).ConvertToQuaternion() * ToMatrix4x4(b).ConvertToQuaternion();
    Quaternion q_from_product = ab.ConvertToQuaternion();
    for (const auto &v : ProbeVectors())
        EXPECT_VEC3_NEAR(q_from_product.RotateVector(v), qab.RotateVector(v), 1e-9);
}

TEST(Rotations, HomogeneousTransformAppliesRotationThenTranslation)
{
    Mat4 t = Mul(Trans(0.1, -0.2, 0.3), AxisAngleMatrix(Vector3(0, 0, 1), M_PI / 2));
    Matrix4x4 m = ToMatrix4x4(t);
    Vector p = m.MultiplyTranspose(Vector({1, 0, 0, 1})); // M * p (see Matrix4x4::MultiplyTranspose)
    EXPECT_NEAR(p.at(0), 0.1, 1e-12);
    EXPECT_NEAR(p.at(1), -0.2 + 1, 1e-12);
    EXPECT_NEAR(p.at(2), 0.3, 1e-12);
    EXPECT_NEAR(p.at(3), 1, 1e-12);
    EXPECT_VEC3_NEAR(m.ConvertToVector3(), Vector3(0.1, -0.2, 0.3), 1e-12);
}

// Vector3::Angle guards acos() against |cos| slightly above 1 with
// std::ceil, which is right for -1.0000001 (-> -1) but maps 1.0000001 to 2,
// so acos returns NaN for (almost) parallel vectors whose normalised dot
// product rounds above 1.
TEST(Rotations, AngleBetweenVectorAndItselfIsZero)
{
    int nan_count = 0;
    for (double x = -2.0; x <= 2.0; x += 0.37)
        for (double y = -2.0; y <= 2.0; y += 0.41)
            for (double z = -2.0; z <= 2.0; z += 0.43)
            {
                Vector3 v(x, y, z);
                if (v.Length() < 1e-6)
                    continue;
                scalar a = v.Angle(v);
                if (std::isnan(a))
                    nan_count++;
                else
                    EXPECT_NEAR(a, 0.0, 1e-6);
            }
    EXPECT_EQ(nan_count, 0) << "Vector3::Angle returned NaN for parallel vectors";
}

TEST(Rotations, AngleBetweenOppositeVectorsIsPi)
{
    for (double x = -2.0; x <= 2.0; x += 0.37)
        for (double y = -2.0; y <= 2.0; y += 0.53)
        {
            Vector3 v(x, y, 0.3);
            EXPECT_NEAR(v.Angle(-v), M_PI, 1e-6);
        }
}
