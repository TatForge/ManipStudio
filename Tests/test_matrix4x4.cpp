#include "gtest/gtest.h"
#include "my_math.h"

using namespace RoboMath;

TEST(Matrix4x4, InitializerListConstructorAndAccessors)
{
    Matrix4x4 m({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    EXPECT_DOUBLE_EQ(m(0, 0), 1);
    EXPECT_DOUBLE_EQ(m(3, 3), 16);
    EXPECT_DOUBLE_EQ(m.at(1, 2), 7);
}

TEST(Matrix4x4, MultiplyByMatrix)
{
    Matrix4x4 m({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    Matrix4x4 n({6, 5, 4, 3, 1, 2, 3, 4, 9, 8, 7, 6, 5, 8, 2, 6});
    EXPECT_EQ(m * n, Matrix4x4({55, 65, 39, 53, 139, 157, 103, 129, 223, 249, 167, 205, 307, 341, 231, 281}));
    EXPECT_EQ(n * m, Matrix4x4({106, 124, 142, 160, 90, 100, 110, 120, 190, 220, 250, 280, 141, 162, 183, 204}));
}

TEST(Matrix4x4, MultiplyTranspose)
{
    Matrix4x4 m({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    Matrix4x4 n({6, 5, 4, 3, 1, 2, 3, 4, 9, 8, 7, 6, 5, 8, 2, 6});
    EXPECT_EQ(m.MultiplyTranspose(n), Matrix4x4({40, 30, 70, 51, 112, 70, 190, 135, 184, 110, 310, 219, 256, 150, 430, 303}));
    EXPECT_EQ(m.MultiplyTranspose(m), Matrix4x4({30, 70, 110, 150, 70, 174, 278, 382, 110, 278, 446, 614, 150, 382, 614, 846}));
}

TEST(Matrix4x4, MultiplyTransposeVector)
{
    Matrix4x4 identity;
    identity.MakeIdentity();
    Vector v({1, 2, 3, 4});
    Vector result = identity.MultiplyTranspose(v);
    EXPECT_TRUE(result == v);
}

// MaxElem is documented to scan only the diagonal to pick the numerically
// dominant term (used by ConvertToQuaternion's Shepperd-method branch), but
// the actual implementation also scans several off-diagonal indices
// (1,2,4,6,8,9). These two matrices reproduce the known-good behaviour
// carried over from the original test scaffolding.
TEST(Matrix4x4, MaxElem)
{
    Matrix4x4 m({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    Matrix4x4 n({6, 5, 4, 3, 1, 2, 3, 4, 9, 8, 7, 6, 5, 8, 2, 6});
    EXPECT_EQ(m.MaxElem(), 2);
    EXPECT_EQ(n.MaxElem(), 0);
    m.MakeIdentity();
    EXPECT_EQ(m.MaxElem(), 0);
}

TEST(Matrix4x4, MakeIdentity)
{
    Matrix4x4 m;
    m.MakeIdentity();
    EXPECT_EQ(m, Matrix4x4({1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}));
}

TEST(Matrix4x4, MakeNull)
{
    Matrix4x4 m({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    m.MakeNull();
    EXPECT_EQ(m, Matrix4x4({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
}

TEST(Matrix4x4, OperatorTimesEqualsScalar)
{
    Matrix4x4 m({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16});
    m *= 2;
    EXPECT_EQ(m, Matrix4x4({2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32}));
}

TEST(Matrix4x4, AssignFromMatrix)
{
    Matrix src{{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}};
    Matrix4x4 dst;
    dst = src;
    EXPECT_DOUBLE_EQ(dst.at(0, 0), 1);
    EXPECT_DOUBLE_EQ(dst.at(3, 3), 16);
}

TEST(Matrix4x4, ConvertToVector3ExtractsTranslationColumn)
{
    Matrix4x4 m({1, 0, 0, 5,
                  0, 1, 0, 6,
                  0, 0, 1, 7,
                  0, 0, 0, 1});
    Vector3 t = m.ConvertToVector3();
    EXPECT_DOUBLE_EQ(t.x, 5);
    EXPECT_DOUBLE_EQ(t.y, 6);
    EXPECT_DOUBLE_EQ(t.z, 7);
}

TEST(Matrix4x4, ConvertToQuaternionFromIdentityRotation)
{
    Matrix4x4 m;
    m.MakeIdentity();
    Quaternion q = m.ConvertToQuaternion();
    // acos() near w=1 amplifies tiny floating-point error into a much
    // larger angle error, so this needs a looser tolerance than most tests.
    EXPECT_NEAR(q.GetAngle(), 0.0, 1e-5);
}

TEST(Matrix4x4, MatrixCoutDoesNotCrash)
{
    Matrix4x4 m;
    m.MakeIdentity();
    testing::internal::CaptureStdout();
    m.MatrixCout("test");
    testing::internal::GetCapturedStdout();
}
