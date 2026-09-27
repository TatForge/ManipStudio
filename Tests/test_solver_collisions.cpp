// Phase 3: collision model of the Solver -- which meshes are loaded, how
// they follow the arm, and that planned motions are stopped by self
// collisions and by the obstacle.

#include "solver_fixture.h"

#include <string>

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

    // Moves the obstacle cube (mesh 8) so that its centre is at `center`.
    void PlaceObstacle(const Vector3 &center)
    {
        Vector3 mn, mx;
        BoundingBox(solver->m_meshes[8], mn, mx);
        Vector3 current = (mn + mx) / 2;
        solver->m_meshes[8].MoveMesh(Quaternion(), center - current);
    }
};

} // namespace

TEST_F(SolverCollisions, LoadsAllNineMeshes)
{
    ASSERT_EQ(solver->m_meshes.size(), 9u);
    for (size_t i = 0; i < solver->m_meshes.size(); i++)
        EXPECT_GE(solver->m_meshes[i].Size(), 4) << "mesh " << i;
}

// Documents a real bug: Solver loads "STLmodels/link3.stl" for mesh 2 (the
// upper arm, between joint 2 and joint 3). The Emulator -- and the file list
// in STLmodels -- use link2.stl there, so the collision model checks the
// upper arm with the forearm's geometry.
TEST_F(SolverCollisions, UpperArmUsesLink2Model)
{
    // (MeshCollider(const std::string&) is declared but never defined, so the
    // file is loaded with ReadStl.)
    MeshCollider link2;
    ASSERT_TRUE(link2.ReadStl("STLmodels/link2.stl"));
    ASSERT_GT(link2.Size(), 0);
    EXPECT_TRUE(SameMesh(solver->m_meshes[2], link2));
}

TEST_F(SolverCollisions, ObstacleIsPlacedAsConfigured)
{
    Vector3 mn, mx;
    BoundingBox(solver->m_meshes[8], mn, mx);
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

TEST_F(SolverCollisions, MoveMeshesKeepsLinksRigid)
{
    Matrix4x4 TR[JOINTS_COUNT];
    solver->SolveDirectKinematic(WorkPose(), TR);
    solver->MoveMeshes(TR);
    for (int m = 0; m < 8; m++)
    {
        const auto &orig = solver->m_meshes[m];
        const auto &moved = solver->m_meshes_buff[m];
        ASSERT_EQ(orig.Size(), moved.Size());
        for (ushort i = 0; i + 1 < orig.Size(); i += 3)
            EXPECT_NEAR(Dist(moved.point(i), moved.point(i + 1)), Dist(orig.point(i), orig.point(i + 1)), 1e-9) << "mesh " << m;
    }
}

// Moving joint 4 must move only what is mounted after it (meshes 6, 7) and
// leave the base, upper arm and forearm meshes where they are.
TEST_F(SolverCollisions, JointMotionMovesOnlyDownstreamMeshes)
{
    Matrix4x4 TR[JOINTS_COUNT];
    Angles q = WorkPose();
    solver->SolveDirectKinematic(q, TR);
    solver->MoveMeshes(TR);
    std::vector<MeshCollider> before = solver->m_meshes_buff;
    q.q[4] += 0.5;
    solver->SolveDirectKinematic(q, TR);
    solver->MoveMeshes(TR);
    for (int m = 0; m <= 5; m++)
        EXPECT_TRUE(SameMesh(before[m], solver->m_meshes_buff[m])) << "mesh " << m << " moved";
    for (int m = 6; m <= 7; m++)
        EXPECT_FALSE(SameMesh(before[m], solver->m_meshes_buff[m])) << "mesh " << m << " did not move";
}

// The tool mesh must travel with the TCP.
TEST_F(SolverCollisions, ToolMeshFollowsTcp)
{
    for (const auto &q : {WorkPose(), MakeAngles({1.1, -0.3, 0.8, 0.6, -0.9, -0.4})})
    {
        Matrix4x4 TR[JOINTS_COUNT];
        solver->SolveDirectKinematic(q, TR);
        solver->MoveMeshes(TR);
        Vector3 mn, mx;
        BoundingBox(solver->m_meshes_buff[7], mn, mx);
        EXPECT_LT(Dist((mn + mx) / 2, ReferenceTcp(q)), 0.2);
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

// Documents a real bug: when mesh i hits the obstacle (mesh 8), the
// short-circuited `||` skips the lambda that assigns j, so the message
// reports whatever j holds: 8 by coincidence if an earlier i ran the lambda
// to completion, an uninitialised value if the very first mesh (the base,
// i = 0) hits the obstacle -- which is the case constructed here.
TEST_F(SolverCollisions, ObstacleCollisionMessageNamesTheObstacle)
{
    PlaceObstacle(Vector3(0, 0, 0.1)); // around the robot base
    std::string msg = CollisionAt(WorkPose());
    ASSERT_NE(msg.find("COLLISION"), std::string::npos) << msg;
    EXPECT_NE(msg.find("Between 0 and 8 "), std::string::npos) << msg;
}
