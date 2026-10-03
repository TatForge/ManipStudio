// Phase 3: collision model of the Solver -- the convex hulls of the arm
// shape (ArmShape) and the obstacle, how they follow the arm, and that
// planned motions are stopped by self collisions and by the obstacle.

#include "solver_fixture.h"

#include <string>
#include <utility>
#include <vector>

using namespace solver_test;

namespace {

class SolverCollisions : public SolverTest
{
protected:
    // "" when pose q is collision free, the exception text otherwise.
    std::string CollisionAt(const Angles &q)
    {
        Matrix4x4 TR[JOINTS_COUNT];
        solver->SolveDirectKinematic(q, TR);
        solver->MoveMeshes(TR);
        uint k = 0;
        try
        {
            ScopedSilence quiet;
            solver->CheckCollision(&k);
        }
        catch (const std::runtime_error &e)
        {
            return e.what();
        }
        return "";
    }

    static void BoundingBox(const MeshCollider &m, Vector3 &mn, Vector3 &mx)
    {
        mn = Vector3(1e9, 1e9, 1e9);
        mx = Vector3(-1e9, -1e9, -1e9);
        for (ushort i = 0; i < m.Size(); i++)
        {
            Vector3 p = m.point(i);
            mn = Vector3(std::min(mn.x, p.x), std::min(mn.y, p.y), std::min(mn.z, p.z));
            mx = Vector3(std::max(mx.x, p.x), std::max(mx.y, p.y), std::max(mx.z, p.z));
        }
    }

    static bool SameMesh(const MeshCollider &a, const MeshCollider &b, double tol = 1e-9)
    {
        if (a.Size() != b.Size())
            return false;
        for (ushort i = 0; i < a.Size(); i++)
            if (Dist(a.point(i), b.point(i)) > tol)
                return false;
        return true;
    }

    MeshCollider &Obstacle() { return solver->m_meshes.back(); }

    // Moves the obstacle cube (the last mesh) so that its centre is at `center`.
    void PlaceObstacle(const Vector3 &center)
    {
        Vector3 mn, mx;
        BoundingBox(Obstacle(), mn, mx);
        Vector3 current = (mn + mx) / 2;
        Obstacle().MoveMesh(Quaternion(), center - current);
    }

    // Replaces the obstacle with a small cube (edge `size`) centred at `center`.
    void PlaceSmallObstacle(const Vector3 &center, double size)
    {
        std::vector<Vector3> corners;
        for (int i = 0; i < 8; i++)
            corners.emplace_back(center.x + (i & 1 ? 0.5 : -0.5) * size, center.y + (i & 2 ? 0.5 : -0.5) * size,
                                 center.z + (i & 4 ? 0.5 : -0.5) * size);
        Obstacle() = MeshCollider(corners);
    }

    const ArmPart *Part(const std::string &name)
    {
        for (const auto &p : solver->GetArmParts())
            if (p.name == name)
                return &p;
        return nullptr;
    }
};

} // namespace

TEST_F(SolverCollisions, OneHullPerArmPartPlusTheObstacle)
{
    const auto &parts = solver->GetArmParts();
    ASSERT_GE(parts.size(), 14u);
    ASSERT_EQ(solver->m_meshes.size(), parts.size() + 1);
    for (size_t i = 0; i < solver->m_meshes.size(); i++)
        EXPECT_GE(solver->m_meshes[i].Size(), 4) << "mesh " << i;
    // the hull of every part is the one ArmShape gives for it
    for (size_t i = 0; i < parts.size(); i++)
    {
        const auto hull = ArmShape::HullPoints(parts[i]);
        ASSERT_EQ(solver->m_meshes[i].Size(), hull.size()) << parts[i].name;
    }
}

// The placed hulls are where the parts are: GUI draws GetArmParts(), the
// check uses the hulls, so both must agree.
TEST_F(SolverCollisions, PlacedHullsSurroundThePartsInWorldCoordinates)
{
    const Angles q = MakeAngles({0.4, -0.7, 1.1, -0.5, 0.8, 0.3});
    Matrix4x4 TR[JOINTS_COUNT];
    solver->SolveDirectKinematic(q, TR);
    solver->MoveMeshes(TR);
    const auto &parts = solver->GetArmParts();
    for (size_t i = 0; i < parts.size(); i++)
    {
        Vector3 from = parts[i].from, to = parts[i].to;
        if (parts[i].body >= 0)
        {
            const Mat4 t = ReferenceT0(q.q, parts[i].body);
            auto map = [&](const Vector3 &v) {
                return Vector3(t[0][0] * v.x + t[0][1] * v.y + t[0][2] * v.z + t[0][3], t[1][0] * v.x + t[1][1] * v.y + t[1][2] * v.z + t[1][3],
                               t[2][0] * v.x + t[2][1] * v.y + t[2][2] * v.z + t[2][3]);
            };
            from = map(from);
            to = map(to);
        }
        // every hull vertex sits about one radius away from the part's axis / centre
        const auto &hull = solver->m_meshes_buff[i];
        for (ushort k = 0; k < hull.Size(); k++)
        {
            const Vector3 v = hull.point(k);
            const double abx = to.x - from.x, aby = to.y - from.y, abz = to.z - from.z;
            const double len2 = abx * abx + aby * aby + abz * abz;
            double t = len2 > 0 ? ((v.x - from.x) * abx + (v.y - from.y) * aby + (v.z - from.z) * abz) / len2 : 0;
            t = std::min(1.0, std::max(0.0, t));
            const double d = Dist(v, Vector3(from.x + t * abx, from.y + t * aby, from.z + t * abz));
            EXPECT_GE(d, parts[i].radius - 1e-6) << parts[i].name;
            EXPECT_LE(d, parts[i].radius * 1.05) << parts[i].name;
        }
    }
}

// The STL collision models left ~15 cm of the upper arm and forearm unchecked.
// A 2 cm obstacle anywhere on the joint-to-joint axis must now be detected.
TEST_F(SolverCollisions, CollisionModelCoversTheWholeArm)
{
    for (const Angles &q : {Angles(), WorkPose()})
    {
        for (int k = 0; k < JOINTS_COUNT - 1; k++)
        {
            const Vector3 a = Position(ReferenceT0(q.q, k)), b = Position(ReferenceT0(q.q, k + 1));
            // the axis runs a(k) along x(k), then d(k+1) along z(k+1)
            const Mat4 tk = ReferenceT0(q.q, k);
            const double ak = ProjectDH()[k].a;
            const Vector3 bend(a.x + tk[0][0] * ak, a.y + tk[1][0] * ak, a.z + tk[2][0] * ak);
            for (const auto &seg : {std::make_pair(a, bend), std::make_pair(bend, b)})
            {
                const double len = Dist(seg.first, seg.second);
                for (double s = 0; s <= len; s += 0.02)
                {
                    const double t = len > 0 ? s / len : 0;
                    const Vector3 x(seg.first.x + t * (seg.second.x - seg.first.x), seg.first.y + t * (seg.second.y - seg.first.y),
                                    seg.first.z + t * (seg.second.z - seg.first.z));
                    PlaceSmallObstacle(x, 0.02);
                    const std::string msg = CollisionAt(q);
                    EXPECT_NE(msg.find("obstacle"), std::string::npos) << "body " << k << " at " << x.x << " " << x.y << " " << x.z << ": '" << msg << "'";
                }
            }
        }
    }
    // the point of the old 8.5 cm gap before the elbow, zero pose
    PlaceSmallObstacle(Vector3(0.30, 0.10, 0.2325), 0.02);
    EXPECT_NE(CollisionAt(Angles()).find("link 2"), std::string::npos);
}

TEST_F(SolverCollisions, ObstacleIsPlacedAsConfigured)
{
    Vector3 mn, mx;
    BoundingBox(Obstacle(), mn, mx);
    EXPECT_VEC3_NEAR((mn + mx) / 2, Vector3(SMESH + 0.05, SMESH - 0.05, 0), 1e-6);
    EXPECT_VEC3_NEAR(mx - mn, Vector3(0.5, 0.5, 0.5), 1e-6);
}

TEST_F(SolverCollisions, WorkingPosesAreCollisionFree)
{
    for (const auto &q : {WorkPose(), solver->GetZeroPosition(), solver->GetInternalPosition(), solver->GetExtarnalPosition()})
        EXPECT_EQ(CollisionAt(q), "") << "q = " << q.q[0] << " " << q.q[1] << " " << q.q[2] << " " << q.q[3] << " " << q.q[4] << " " << q.q[5];
}

TEST_F(SolverCollisions, FoldedElbowIsASelfCollision)
{
    std::string msg = CollisionAt(MakeAngles({0, 0, 3.0, 0, 0, 0}));
    EXPECT_NE(msg.find("COLLISION"), std::string::npos) << msg;
}

// ---------------------------------------------------------------------------
// Floor / table plane
// ---------------------------------------------------------------------------

TEST_F(SolverCollisions, FloorIsAtMountingPlaneByDefault)
{
    EXPECT_DOUBLE_EQ(solver->GetFloorHeight(), robot->GetFloorHeight());
    EXPECT_DOUBLE_EQ(solver->GetFloorHeight(), 0.0);
}

// Shoulder lowered to q1 = -0.1 rad: the tool dips ~7 cm below the table.
TEST_F(SolverCollisions, LinkBelowFloorIsACollision)
{
    std::string msg = CollisionAt(MakeAngles({0.3, -0.1, 0.9, 0.2, -0.4, 0.1}));
    EXPECT_NE(msg.find("COLLISION"), std::string::npos) << msg;
    EXPECT_NE(msg.find("floor"), std::string::npos) << msg;
}

TEST_F(SolverCollisions, FloorHeightIsConfigurable)
{
    solver->SetFloorHeight(-0.1);
    EXPECT_EQ(CollisionAt(MakeAngles({0.3, -0.1, 0.9, 0.2, -0.4, 0.1})), "");
}

// The base itself stands on the floor; that must not count as a collision.
TEST_F(SolverCollisions, BaseStandingOnFloorIsNotACollision)
{
    EXPECT_EQ(CollisionAt(Angles()), "");
}

TEST_F(SolverCollisions, MoveJIntoTheTableIsRejected)
{
    Angles to = WorkPose();
    to.q[1] = -0.1;
    std::vector<float> flat;
    try
    {
        ScopedSilence quiet;
        solver->MoveJ(WorkPose(), to, 0.5f, 1.0f, 10.0f, flat);
        ADD_FAILURE() << "MoveJ into the table was planned";
    }
    catch (const std::runtime_error &e)
    {
        EXPECT_NE(std::string(e.what()).find("floor"), std::string::npos) << e.what();
    }
}

TEST_F(SolverCollisions, MoveMeshesKeepsPartsRigid)
{
    Matrix4x4 TR[JOINTS_COUNT];
    solver->SolveDirectKinematic(WorkPose(), TR);
    solver->MoveMeshes(TR);
    for (size_t m = 0; m < solver->GetArmParts().size(); m++)
    {
        const auto &orig = solver->m_meshes[m];
        const auto &moved = solver->m_meshes_buff[m];
        ASSERT_EQ(orig.Size(), moved.Size());
        for (ushort i = 0; i + 3 < orig.Size(); i += 3)
            EXPECT_NEAR(Dist(moved.point(i), moved.point(i + 3)), Dist(orig.point(i), orig.point(i + 3)), 1e-9) << "mesh " << m;
    }
}

// Moving joint 5 (q[4]) must move only what is mounted from frame 4 on and
// leave the base, the upper arm and the forearm where they are.
TEST_F(SolverCollisions, JointMotionMovesOnlyDownstreamParts)
{
    Matrix4x4 TR[JOINTS_COUNT];
    Angles q = WorkPose();
    solver->SolveDirectKinematic(q, TR);
    solver->MoveMeshes(TR);
    std::vector<MeshCollider> before = solver->m_meshes_buff;
    q.q[4] += 0.5;
    solver->SolveDirectKinematic(q, TR);
    solver->MoveMeshes(TR);
    const auto &parts = solver->GetArmParts();
    for (size_t m = 0; m < parts.size(); m++)
    {
        // the drum of joint 5 turns about its own axis: its hull vertices move too
        if (parts[m].body < 4)
            EXPECT_TRUE(SameMesh(before[m], solver->m_meshes_buff[m])) << parts[m].name << " moved";
        else
            EXPECT_FALSE(SameMesh(before[m], solver->m_meshes_buff[m])) << parts[m].name << " did not move";
    }
}

// The flange must travel with the TCP.
TEST_F(SolverCollisions, FlangeFollowsTcp)
{
    const auto &parts = solver->GetArmParts();
    size_t flange = parts.size();
    for (size_t i = 0; i < parts.size(); i++)
        if (parts[i].name == "flange")
            flange = i;
    ASSERT_LT(flange, parts.size());
    for (const auto &q : {WorkPose(), MakeAngles({1.1, -0.3, 0.8, 0.6, -0.9, -0.4})})
    {
        Matrix4x4 TR[JOINTS_COUNT];
        solver->SolveDirectKinematic(q, TR);
        solver->MoveMeshes(TR);
        Vector3 mn, mx;
        BoundingBox(solver->m_meshes_buff[flange], mn, mx);
        EXPECT_LT(Dist((mn + mx) / 2, ReferenceTcp(q)), 0.06);
    }
}

// High-level: a joint move that folds the elbow into the upper arm must be
// rejected with a collision error (not planned, and not rejected for some
// other reason).
TEST_F(SolverCollisions, MoveJIntoSelfCollisionIsRejected)
{
    Angles from = WorkPose(), to = WorkPose();
    to.q[2] = 3.0;
    std::vector<float> flat;
    try
    {
        ScopedSilence quiet;
        solver->MoveJ(from, to, 0.5f, 1.0f, 10.0f, flat);
        ADD_FAILURE() << "MoveJ into a folded elbow was planned without error";
    }
    catch (const std::runtime_error &e)
    {
        EXPECT_NE(std::string(e.what()).find("COLLISION"), std::string::npos) << e.what();
    }
}

// High-level: an obstacle placed on the path of a MoveL must stop it.
// (The configured obstacle at (1.7, 1.6, 0) is far outside the ~0.8 m reach,
// so it is moved onto the path here.)
TEST_F(SolverCollisions, MoveLThroughObstacleIsRejected)
{
    Angles start = WorkPose();
    StateVector p1 = ReferencePose(start), p2 = p1;
    p2.x -= 0.10;
    PlaceObstacle(Vector3(p1.x - 0.05, p1.y, p1.z - 0.3)); // 0.5 m cube touching the path from below
    std::vector<float> flat;
    try
    {
        ScopedSilence quiet;
        solver->MoveL(p1, p2, 0.1, 0.5, start, 10, flat);
        ADD_FAILURE() << "MoveL through the obstacle was planned without error";
    }
    catch (const std::runtime_error &e)
    {
        EXPECT_NE(std::string(e.what()).find("COLLISION"), std::string::npos) << e.what();
    }
}

// Regression test for a fixed bug (the message once named an uninitialised
// part when the base hit the obstacle); parts are now named.
TEST_F(SolverCollisions, ObstacleCollisionMessageNamesBothParts)
{
    PlaceObstacle(Vector3(0, 0, 0.1)); // around the robot base
    std::string msg = CollisionAt(WorkPose());
    ASSERT_NE(msg.find("COLLISION"), std::string::npos) << msg;
    EXPECT_NE(msg.find("Between base and obstacle"), std::string::npos) << msg;
}

TEST_F(SolverCollisions, SelfCollisionMessageNamesBothParts)
{
    std::string msg = CollisionAt(MakeAngles({0, 0, 3.0, 0, 0, 0}));
    ASSERT_NE(msg.find("Between "), std::string::npos) << msg;
    EXPECT_EQ(msg.find("obstacle"), std::string::npos) << msg;
}

TEST_F(SolverCollisions, FloorMessageNamesThePart)
{
    std::string msg = CollisionAt(MakeAngles({0.3, -0.1, 0.9, 0.2, -0.4, 0.1}));
    EXPECT_NE(msg.find("below the floor"), std::string::npos) << msg;
    EXPECT_TRUE(msg.find("joint") != std::string::npos || msg.find("link") != std::string::npos || msg.find("flange") != std::string::npos) << msg;
}
