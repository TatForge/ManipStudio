// ArmShape: the arm geometry built from the DH table. Checked against the
// independent reference kinematics of test_helpers.h: the parts form an
// unbroken chain from the floor to the flange, and their hulls contain them.

#include "test_helpers.h"

#include "armshape.h"

#include <cmath>
#include <string>
#include <vector>

using namespace RoboMath;
using namespace test_helpers;

namespace {

Vector3 Transform(const Mat4 &m, const Vector3 &p)
{
    return Vector3(m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3],
                   m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
                   m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3]);
}

// part axis end points in world coordinates for joint angles q
void World(const ArmPart &p, const double q[JOINTS_COUNT], Vector3 &from, Vector3 &to)
{
    if (p.body < 0)
    {
        from = p.from;
        to = p.to;
        return;
    }
    const Mat4 t = ReferenceT0(q, p.body);
    from = Transform(t, p.from);
    to = Transform(t, p.to);
}

std::vector<ArmPart> Links(const std::vector<ArmPart> &parts, int body)
{
    std::vector<ArmPart> out;
    for (const auto &p : parts)
        if (p.body == body && p.kind == ArmPart::Cylinder && p.name.rfind("link", 0) == 0)
            out.push_back(p);
    return out;
}

// distance from point x to segment a-b
double SegmentDistance(const Vector3 &x, const Vector3 &a, const Vector3 &b)
{
    const double abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    const double len2 = abx * abx + aby * aby + abz * abz;
    double t = len2 > 0 ? ((x.x - a.x) * abx + (x.y - a.y) * aby + (x.z - a.z) * abz) / len2 : 0;
    t = std::min(1.0, std::max(0.0, t));
    return Dist(x, Vector3(a.x + t * abx, a.y + t * aby, a.z + t * abz));
}

const std::vector<std::vector<double>> &Poses()
{
    static const std::vector<std::vector<double>> poses{
        {0, 0, 0, 0, 0, 0}, {0, -M_PI / 4, M_PI / 2, M_PI / 4, M_PI / 2, 0}, {0.3, -0.5, 0.9, 0.2, -0.4, 0.1}, {-2.1, -1.7, -1.0, 2.6, 1.9, -5.2}};
    return poses;
}

} // namespace

TEST(ArmShape, LinksConnectEveryJointInEveryPose)
{
    const auto parts = ArmShape::Build(ProjectDHMatrix());
    for (const auto &pose : Poses())
    {
        for (int k = 0; k < JOINTS_COUNT - 1; k++)
        {
            const auto links = Links(parts, k);
            ASSERT_FALSE(links.empty()) << "body " << k;
            Vector3 from, to, prev_to;
            World(links.front(), pose.data(), from, to);
            EXPECT_VEC3_NEAR(from, Position(ReferenceT0(pose.data(), k)), 1e-9);
            for (size_t i = 0; i < links.size(); i++)
            {
                World(links[i], pose.data(), from, to);
                if (i > 0)
                    EXPECT_VEC3_NEAR(from, prev_to, 1e-9);
                prev_to = to;
            }
            EXPECT_VEC3_NEAR(prev_to, Position(ReferenceT0(pose.data(), k + 1)), 1e-9) << "body " << k;
        }
    }
}

// The STL collision models were shorter than the links (8.5 cm gap before the elbow).
TEST(ArmShape, UpperArmAndForearmHaveTheDhLengths)
{
    const auto parts = ArmShape::Build(ProjectDHMatrix());
    const auto upper = Links(parts, 1), fore = Links(parts, 2);
    ASSERT_FALSE(upper.empty());
    ASSERT_FALSE(fore.empty());
    EXPECT_NEAR(Dist(upper.front().from, upper.front().to), 0.375, 1e-12);
    EXPECT_NEAR(Dist(fore.front().from, fore.front().to), 0.295, 1e-12);
}

TEST(ArmShape, EveryJointHasADrumOnItsAxis)
{
    const auto parts = ArmShape::Build(ProjectDHMatrix());
    int drums = 0;
    for (const auto &p : parts)
    {
        if (p.name.rfind("joint", 0) != 0)
            continue;
        drums++;
        EXPECT_EQ(p.name, "joint " + std::to_string(p.body + 1));
        EXPECT_DOUBLE_EQ(p.from.x, 0);
        EXPECT_DOUBLE_EQ(p.from.y, 0);
        EXPECT_DOUBLE_EQ(p.to.x, 0);
        EXPECT_DOUBLE_EQ(p.to.y, 0);
        EXPECT_NEAR(p.from.z, -p.to.z, 1e-12);       // centred on the frame origin
        EXPECT_DOUBLE_EQ(p.radius, ArmShape::DefaultJointRadii()[p.body]);
    }
    EXPECT_EQ(drums, JOINTS_COUNT);
}

TEST(ArmShape, BaseStandsOnTheFloorBelowTheFirstJoint)
{
    const auto parts = ArmShape::Build(ProjectDHMatrix());
    bool found = false;
    for (const auto &p : parts)
        if (p.body < 0)
        {
            found = true;
            EXPECT_EQ(p.name, "base");
            EXPECT_DOUBLE_EQ(p.from.z, 0);
            EXPECT_LT(p.to.z, ProjectDH()[0].d);
        }
    EXPECT_TRUE(found);
}

TEST(ArmShape, JointRadiiAreConfigurable)
{
    std::array<scalar, JOINTS_COUNT> radii{{0.1, 0.09, 0.08, 0.07, 0.06, 0.05}};
    for (const auto &p : ArmShape::Build(ProjectDHMatrix(), radii))
        if (p.name.rfind("joint", 0) == 0)
            EXPECT_DOUBLE_EQ(p.radius, radii[p.body]);
}

TEST(ArmShape, CylinderHullContainsTheCylinder)
{
    const ArmPart p{ArmPart::Cylinder, 0, Vector3(0.1, -0.2, 0.3), Vector3(0.4, 0.1, 0.5), 0.05, "link 1"};
    const auto hull = ArmShape::HullPoints(p, 16);
    ASSERT_EQ(hull.size(), 32u);
    for (const auto &v : hull)
    {
        // on the end circles, slightly outside the radius (circumscribed polygon)
        const double d = SegmentDistance(v, p.from, p.to);
        EXPECT_GE(d, p.radius - 1e-12);
        EXPECT_LE(d, p.radius / std::cos(M_PI / 16) + 1e-12);
    }
}

TEST(ArmShape, SphereHullContainsTheSphere)
{
    const ArmPart p{ArmPart::Sphere, 2, Vector3(0.2, 0, 0), Vector3(0.2, 0, 0), 0.04, "link 3"};
    const auto hull = ArmShape::HullPoints(p, 16);
    ASSERT_GT(hull.size(), 20u);
    for (const auto &v : hull)
        EXPECT_GE(Dist(v, p.from), p.radius);
}

TEST(ArmShape, OnlyNeighbouringBodiesAreAdjacent)
{
    EXPECT_TRUE(ArmShape::Adjacent(-1, 0));
    EXPECT_TRUE(ArmShape::Adjacent(3, 3));
    EXPECT_TRUE(ArmShape::Adjacent(4, 3));
    EXPECT_FALSE(ArmShape::Adjacent(-1, 1));
    EXPECT_FALSE(ArmShape::Adjacent(1, 3));
}
