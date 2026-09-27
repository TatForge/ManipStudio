#include "gtest/gtest.h"
#include "meshcollider.h"

#include <cmath>
#include <string>
#include <vector>

using namespace RoboMath;

namespace {
std::string CubeModelPath()
{
    return std::string(TEST_MODELS_DIR) + "cube2m.stl";
}
} // namespace

TEST(MeshCollider, ReadStlLoadsKnownCube)
{
    MeshCollider mc;
    EXPECT_TRUE(mc.ReadStl(CubeModelPath()));
    // cube2m.stl is a 12-triangle (36 raw-vertex) axis-aligned 2m cube
    // centered on the origin; after de-duplication only its 8 corners
    // should remain.
    EXPECT_EQ(mc.Size(), 8);
}

TEST(MeshCollider, ReadStlThrowsOnMissingFile)
{
    MeshCollider mc;
    EXPECT_THROW(mc.ReadStl(std::string(TEST_MODELS_DIR) + "does_not_exist.stl"), std::runtime_error);
}

TEST(MeshCollider, PointReturnsMeshVertex)
{
    MeshCollider mc;
    mc.ReadStl(CubeModelPath());
    for (ushort i = 0; i < mc.Size(); i++)
    {
        Vector3 p = mc.point(i);
        EXPECT_NEAR(std::abs(p.x), 1.0, 1e-9);
        EXPECT_NEAR(std::abs(p.y), 1.0, 1e-9);
        EXPECT_NEAR(std::abs(p.z), 1.0, 1e-9);
    }
}

TEST(MeshCollider, MoveMeshInPlaceTranslatesPoints)
{
    MeshCollider mc;
    mc.ReadStl(CubeModelPath());
    Vector3 before = mc.point(0);
    mc.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(5, 0, 0)); // identity rotation
    Vector3 after = mc.point(0);
    EXPECT_NEAR(after.x, before.x + 5, 1e-9);
    EXPECT_NEAR(after.y, before.y, 1e-9);
    EXPECT_NEAR(after.z, before.z, 1e-9);
}

TEST(MeshCollider, MoveMeshConstOverloadLeavesOriginalUnchanged)
{
    MeshCollider mc;
    mc.ReadStl(CubeModelPath());
    Vector3 before = mc.point(0);
    // Overload resolution picks the in-place bool-returning overload for a
    // non-const object, so the copying const overload must be invoked
    // through a const reference.
    const MeshCollider &const_mc = mc;
    MeshCollider moved = const_mc.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(5, 0, 0));
    EXPECT_NEAR(mc.point(0).x, before.x, 1e-9); // original untouched
    EXPECT_NEAR(moved.point(0).x, before.x + 5, 1e-9);
}

TEST(MeshCollider, CheckCollisionSequence)
{
    MeshCollider mc1;
    MeshCollider mc2;

    EXPECT_TRUE(mc1.ReadStl(CubeModelPath()));
    EXPECT_TRUE(mc2.ReadStl(CubeModelPath()));
    EXPECT_TRUE(mc1.CheckCollision(mc2));

    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(1, 1, 0));
    EXPECT_TRUE(mc1.CheckCollision(mc2));

    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(1, 1, 0));
    EXPECT_TRUE(mc1.CheckCollision(mc2));

    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(0.1, 0.1, 0));
    EXPECT_FALSE(mc1.CheckCollision(mc2));

    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(-2.1, -2.1, 0));
    EXPECT_TRUE(mc1.CheckCollision(mc2));

    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(-2.0, -2.0, 0));
    EXPECT_TRUE(mc1.CheckCollision(mc2));

    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(-0.1, -0.1, 0));
    EXPECT_FALSE(mc1.CheckCollision(mc2));

    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(-11.1, -11.1, 0));
    EXPECT_FALSE(mc1.CheckCollision(mc2));

    // move back to zero position
    mc2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(11.2, 11.2, 0));
    EXPECT_TRUE(mc1.CheckCollision(mc2));

    // rotate 90 degrees around Z then translate away
    mc2.MoveMesh(Quaternion(0.0, 0, 0.707, 0.707), Vector3(2, 2, 0));
    EXPECT_FALSE(mc1.CheckCollision(mc2));
}

// ---------------------------------------------------------------------------
// GJK collision properties (regression net for any change to MeshCollider)
// ---------------------------------------------------------------------------

namespace {

MeshCollider Cube2m()
{
    MeshCollider mc;
    mc.ReadStl(CubeModelPath());
    return mc;
}

MeshCollider MovedCopy(const MeshCollider &mc, const Quaternion &rot, const Vector3 &offset)
{
    return mc.MoveMesh(rot, offset); // const overload -> copy
}

} // namespace

// Two axis-aligned 2 m cubes overlap exactly when every centre offset
// component is below 2 m. Checked on a grid of offsets (avoiding the exact
// touching boundary, where the answer is a matter of convention).
// Currently fails for offset (0.4, -2.6, -1.5): the cubes are 0.6 m apart
// in Y, yet GJK reports a collision (false positive in the simplex
// handling).
TEST(MeshColliderGjk, AxisAlignedCubesMatchAnalyticOverlap)
{
    const MeshCollider a = Cube2m();
    const std::vector<double> offs{-2.6, -2.05, -1.95, -1.2, -0.3, 0.0, 0.4, 1.1, 1.95, 2.05, 2.7};
    int checked = 0;
    for (double dx : offs)
        for (double dy : offs)
            for (double dz : {-2.3, -1.5, 0.0, 0.9, 1.97, 2.02})
            {
                MeshCollider a_copy = a;
                MeshCollider b = MovedCopy(a, Quaternion(), Vector3(dx, dy, dz));
                bool expected = std::abs(dx) < 2 && std::abs(dy) < 2 && std::abs(dz) < 2;
                EXPECT_EQ(a_copy.CheckCollision(b), expected) << "offset " << dx << ", " << dy << ", " << dz;
                checked++;
            }
    EXPECT_GT(checked, 500);
}

TEST(MeshColliderGjk, CollisionIsSymmetric)
{
    const MeshCollider base = Cube2m();
    for (double dx : {-2.2, -1.0, 0.5, 1.9, 2.4})
        for (double dy : {-1.9, 0.0, 2.1})
        {
            MeshCollider a = base;
            MeshCollider b = MovedCopy(base, Quaternion(Vector3(0.3, 1, 0.2), 0.7), Vector3(dx, dy, 0.3));
            EXPECT_EQ(a.CheckCollision(b), b.CheckCollision(a)) << dx << ", " << dy;
        }
}

// A cube rotated 45 deg about Z reaches sqrt(2) m from its centre along X.
TEST(MeshColliderGjk, RotatedCubeUsesItsRealExtent)
{
    const MeshCollider a = Cube2m();
    Quaternion rot45(unit_z, M_PI / 4);
    MeshCollider a1 = a, a2 = a;
    MeshCollider near = MovedCopy(a, rot45, Vector3(2.3, 0, 0)); // corner reaches x = 0.886 < 1
    MeshCollider far = MovedCopy(a, rot45, Vector3(2.5, 0, 0));  // corner reaches x = 1.086 > 1
    EXPECT_TRUE(a1.CheckCollision(near));
    EXPECT_FALSE(a2.CheckCollision(far));
}

TEST(MeshColliderGjk, ContainedObjectCollides)
{
    MeshCollider big = Cube2m(), small;
    ASSERT_TRUE(small.ReadStl(std::string(TEST_PROJECT_DIR) + "STLmodels/cube0.5m.stl"));
    MeshCollider inside = MovedCopy(small, Quaternion(Vector3(1, 1, 0), 0.4), Vector3(0.2, -0.3, 0.1));
    EXPECT_TRUE(big.CheckCollision(inside));
    EXPECT_TRUE(inside.CheckCollision(big));
}

TEST(MeshColliderGjk, MoveMeshRotatesAboutOriginThenTranslates)
{
    const MeshCollider a = Cube2m();
    Quaternion rot(unit_z, M_PI / 2);
    MeshCollider b = MovedCopy(a, rot, Vector3(10, 0, 0));
    ASSERT_EQ(a.Size(), b.Size());
    for (ushort i = 0; i < a.Size(); i++)
    {
        Vector3 expected = rot.RotateVector(a.point(i)) + Vector3(10, 0, 0);
        EXPECT_NEAR((b.point(i) - expected).Length(), 0, 1e-9);
    }
}

TEST(MeshCollider, RemoveDublicateVertexesRemovesRepeats)
{
    // Loading the cube already exercises RemoveDublicateVertexes on real
    // data: 36 raw STL vertices collapse to the cube's 8 unique corners.
    MeshCollider mc;
    mc.ReadStl(CubeModelPath());
    EXPECT_EQ(mc.Size(), 8);
}

// Documents a real bug: RemoveDublicateVertexes computes `m_size - 1` where
// m_size is an unsigned ushort. On an empty mesh (m_size == 0) this
// underflows to a huge value, and the subsequent vector access is out of
// bounds. EXPECT_DEATH runs the call in a forked subprocess so a crash here
// doesn't take down the rest of the test suite.
TEST(MeshColliderDeathTest, RemoveDublicateVertexesOnEmptyMeshIsUnsafe)
{
    EXPECT_DEATH(
        {
            MeshCollider mc;
            mc.RemoveDublicateVertexes();
        },
        "");
}
