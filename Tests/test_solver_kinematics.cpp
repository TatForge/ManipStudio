// Phase 3: Solver kinematics -- forward kinematics against the independent
// reference, inverse kinematics round trips from various initial conditions,
// and the small helper functions.

#include "solver_fixture.h"

using namespace solver_test;

namespace {

const std::vector<Angles> &Poses()
{
    static const std::vector<Angles> poses{
        MakeAngles({0, 0, 0, 0, 0, 0}),
        WorkPose(),
        MakeAngles({0, -M_PI / 4, M_PI / 2, M_PI / 4, M_PI / 2, 0}),               // GetInternalPosition
        MakeAngles({0, -M_PI / 4, M_PI / 2, -M_PI + M_PI / 3.7, -M_PI / 2, 0}),    // GetExtarnalPosition
        MakeAngles({2.0, 0.4, -1.1, 1.5, 0.8, -2.2}),
        MakeAngles({-0.7, -1.0, 1.4, -0.6, 1.2, 0.9})};
    return poses;
}

// Goal configurations for IK round trips: bent elbow, wrist away from
// singularities, generic TCP orientation.
const std::vector<Angles> &IkGoals()
{
    static const std::vector<Angles> goals{
        WorkPose(),
        MakeAngles({-0.4, -0.6, 1.2, -0.3, 0.7, 0.5}),
        MakeAngles({1.1, -0.3, 0.8, 0.6, -0.9, -0.4}),
        MakeAngles({0.0, -0.9, 1.6, 0.4, 0.6, 0.2}),
        MakeAngles({-1.3, -0.2, 0.6, -0.8, -0.5, 1.0})};
    return goals;
}

} // namespace

TEST_F(SolverTest, DirectKinematicMatchesReferenceForEveryFrame)
{
    for (const auto &q : Poses())
    {
        Matrix4x4 TR[JOINTS_COUNT];
        solver->SolveDirectKinematic(q, TR);
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            Mat4 ref = ReferenceT0(q.q, j);
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    ASSERT_NEAR(TR[j].at(r, c), ref[r][c], 1e-12) << "frame " << j << " (" << r << "," << c << ")";
        }
    }
}

TEST_F(SolverTest, DirectKinematicWritesLocalLinkTransforms)
{
    Angles q = WorkPose();
    Matrix4x4 TR[JOINTS_COUNT], T[JOINTS_COUNT];
    solver->SolveDirectKinematic(q, TR, T);
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        Mat4 ref = LinkTransform(j, q.q[j]);
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                EXPECT_NEAR(T[j].at(r, c), ref[r][c], 1e-12) << "link " << j;
    }
}

TEST_F(SolverTest, DirectKinematicZeroPoseMatchesHandCalculation)
{
    Matrix4x4 TR[JOINTS_COUNT];
    solver->SolveDirectKinematic(Angles(), TR);
    EXPECT_VEC3_NEAR(Position(TR[5]), Vector3(0.67, 0.2431, 0.0615), 1e-12);
}

TEST_F(SolverTest, DirectKinematicAgreesWithRoboForwardKinematic)
{
    Matrix dh = ProjectDHMatrix();
    for (const auto &q : Poses())
    {
        Matrix4x4 TR[JOINTS_COUNT];
        solver->SolveDirectKinematic(q, TR);
        EXPECT_VEC3_NEAR(Position(TR[5]), robot->SolveForwardKinematic(q, &dh).GetCoord(), 1e-12);
    }
}

// Rotating joint 0 (the vertical base axis) must rotate the TCP about the
// world Z axis and leave its height and distance from the axis unchanged.
TEST_F(SolverTest, BaseJointRotatesTcpAboutVerticalAxis)
{
    Angles q = WorkPose();
    Vector3 p0 = SolverPose(q).GetCoord();
    for (double d : {-1.0, 0.25, 1.7})
    {
        Angles q2 = q;
        q2.q[0] += d;
        Vector3 p = SolverPose(q2).GetCoord();
        EXPECT_NEAR(p.z, p0.z, 1e-12);
        EXPECT_NEAR(std::hypot(p.x, p.y), std::hypot(p0.x, p0.y), 1e-12);
        EXPECT_NEAR(AngleDiff(atan2(p.y, p.x), atan2(p0.y, p0.x)), d, 1e-12);
    }
}

// The roll/pitch/yaw reported for the robot's predefined start poses must be
// its real orientation. The zero pose and GetInternalPosition() have the
// tool pointing straight down with a half-turn rotation matrix (trace = -1);
// for GetInternalPosition() rounding sends Matrix4x4::ConvertToQuaternion
// into its defective branch and the reported orientation is off by exactly
// 180 deg -- every MoveL / MoveC started from there gets a wrong start
// orientation.
TEST_F(SolverTest, ReportedOrientationOfPredefinedPosesIsCorrect)
{
    for (const auto &q : {solver->GetZeroPosition(), solver->GetInternalPosition(), solver->GetExtarnalPosition(), WorkPose()})
    {
        Mat4 reported = EulerMatrix(SolverPose(q).GetAngles());
        EXPECT_NEAR(RotationDistance(reported, ReferenceTcpFrame(q)), 0, 1e-6)
            << "q = " << q.q[0] << " " << q.q[1] << " " << q.q[2] << " " << q.q[3] << " " << q.q[4] << " " << q.q[5];
    }
}

TEST_F(SolverTest, InverseKinematicAtCurrentPoseKeepsAngles)
{
    for (const auto &q : IkGoals())
    {
        Angles res;
        ASSERT_TRUE(RunsWithoutThrowing([&] { solver->SolveInverseKinematic(SolverPose(q), q, res); }));
        for (int j = 0; j < JOINTS_COUNT; j++)
            EXPECT_NEAR(res.q[j], q.q[j], 1e-6) << "joint " << j;
    }
}

// High-level IK behaviour: starting from a perturbed configuration the solver
// must reach the requested TCP pose. Checked with the independent reference
// FK, for several goals and perturbation sizes.
class SolverIkRoundTrip : public SolverTest, public ::testing::WithParamInterface<double>
{
};

TEST_P(SolverIkRoundTrip, ReachesTargetPoseFromPerturbedStart)
{
    const double perturbation = GetParam();
    for (size_t g = 0; g < IkGoals().size(); g++)
    {
        const Angles &goal = IkGoals()[g];
        Angles start = goal;
        for (int j = 0; j < JOINTS_COUNT; j++)
            start.q[j] += perturbation * ((j % 2) ? 1 : -1) * (1 + 0.3 * j);
        StateVector target = ReferencePose(goal);
        Angles res;
        ASSERT_TRUE(RunsWithoutThrowing([&] { solver->SolveInverseKinematic(target, start, res); })) << "goal " << g;
        EXPECT_VEC3_NEAR(ReferenceTcp(res), target.GetCoord(), 1e-6) << "goal " << g;
        EXPECT_NEAR(RotationDistance(ReferenceTcpFrame(res), ReferenceTcpFrame(goal)), 0, 1e-5) << "goal " << g;
    }
}

INSTANTIATE_TEST_SUITE_P(Perturbations, SolverIkRoundTrip, ::testing::Values(0.01, 0.05, 0.15));

// Small Cartesian steps (what MoveL/MoveC ask for every 10 ms) must produce
// small joint steps -- the IK must stay on the current solution branch.
TEST_F(SolverTest, InverseKinematicSmallStepGivesSmallJointChange)
{
    for (const auto &q : IkGoals())
    {
        StateVector target = ReferencePose(q);
        target.x += 0.001;
        target.z -= 0.001;
        Angles res;
        ASSERT_TRUE(RunsWithoutThrowing([&] { solver->SolveInverseKinematic(target, q, res); }));
        for (int j = 0; j < JOINTS_COUNT; j++)
            EXPECT_LT(std::abs(res.q[j] - q.q[j]), 0.05) << "joint " << j;
    }
}

// Same requirement starting from GetInternalPosition(): the start
// orientation is taken from the Solver's own (buggy at this pose) FK, so a
// 1 mm step makes the IK jump to a completely different arm configuration.
TEST_F(SolverTest, InverseKinematicSmallStepFromInternalPositionStaysOnBranch)
{
    Angles q = solver->GetInternalPosition();
    StateVector target = SolverPose(q);
    target.x += 0.001;
    Angles res;
    ASSERT_TRUE(RunsWithoutThrowing([&] { solver->SolveInverseKinematic(target, q, res); }));
    for (int j = 0; j < JOINTS_COUNT; j++)
        EXPECT_LT(std::abs(res.q[j] - q.q[j]), 0.05) << "joint " << j;
}

// Near singular configurations the DLS solver must either reach the target
// or give up with an exception -- never return NaN / garbage angles.
TEST_F(SolverTest, InverseKinematicNearSingularitiesIsWellBehaved)
{
    const std::vector<Angles> singular_goals{
        MakeAngles({0.2, -0.6, 1.1, 0.3, 0.0, 0.4}),   // wrist: joints 3 and 5 aligned
        MakeAngles({0.2, -0.6, 1.1, 0.3, 1e-3, 0.4}),  // just off the wrist singularity
        MakeAngles({0.1, -0.4, 0.02, 0.2, -0.5, 0.1}), // elbow almost straight
    };
    for (size_t g = 0; g < singular_goals.size(); g++)
    {
        Angles start = singular_goals[g];
        for (int j = 0; j < JOINTS_COUNT; j++)
            start.q[j] += 0.03 * ((j % 2) ? 1 : -1);
        StateVector target = ReferencePose(singular_goals[g]);
        Angles res;
        try
        {
            ScopedSilence quiet;
            solver->SolveInverseKinematic(target, start, res);
        }
        catch (const std::runtime_error &)
        {
            continue; // giving up is acceptable
        }
        for (int j = 0; j < JOINTS_COUNT; j++)
            ASSERT_TRUE(std::isfinite(res.q[j])) << "goal " << g << " joint " << j;
        EXPECT_VEC3_NEAR(ReferenceTcp(res), target.GetCoord(), 1e-5) << "goal " << g;
    }
}

TEST_F(SolverTest, InverseKinematicThrowsForUnreachableTarget)
{
    StateVector far_away(2.0, 2.0, 2.0, 0.0, 0.0, 0.0);
    Angles res;
    ScopedSilence quiet;
    EXPECT_THROW(solver->SolveInverseKinematic(far_away, WorkPose(), res), std::runtime_error);
}

TEST_F(SolverTest, InverseKinematicLeavesResultUntouchedWhenItThrows)
{
    Angles res = WorkPose();
    try
    {
        ScopedSilence quiet;
        solver->SolveInverseKinematic(StateVector(2.0, 2.0, 2.0, 0.0, 0.0, 0.0), WorkPose(), res);
    }
    catch (const std::runtime_error &)
    {
    }
    for (int j = 0; j < JOINTS_COUNT; j++)
        EXPECT_DOUBLE_EQ(res.q[j], WorkPose().q[j]);
}

TEST_F(SolverTest, InverseKinematicUsesProvidedIkDataBuffer)
{
    IKData data;
    Angles res;
    StateVector target = ReferencePose(IkGoals()[1]);
    ASSERT_TRUE(RunsWithoutThrowing([&] { solver->SolveInverseKinematic(target, WorkPose(), res, &data); }));
    EXPECT_GT(data.ctr, 0);
    EXPECT_LE(data.ctr, MAX_KINEMATIC_ITERS);
    EXPECT_VEC3_NEAR(Position(data.TR[5]), target.GetCoord(), 1e-6);
}

TEST_F(SolverTest, CheckTrajectoryForLinearAcceptsPointOnLine)
{
    Angles q = WorkPose();
    Vector3 p = ReferenceTcp(q);
    ScopedSilence quiet;
    EXPECT_TRUE(solver->CheckTrajectoryForLinear(q, p - Vector3(0.1, 0.2, 0), p + Vector3(0.1, 0.2, 0), DELTA_RANGE));
}

// Off the line CheckTrajectoryForLinear does not return false: it throws a
// QString (the `return false` after the throw is unreachable).
TEST_F(SolverTest, CheckTrajectoryForLinearThrowsQStringOffLine)
{
    Angles q = WorkPose();
    Vector3 p = ReferenceTcp(q) + Vector3(0, 0, 0.01);
    ScopedSilence quiet;
    EXPECT_THROW(solver->CheckTrajectoryForLinear(q, p - Vector3(0.1, 0, 0), p + Vector3(0.1, 0, 0), DELTA_RANGE), QString);
}

TEST_F(SolverTest, AnglesVectorRoundTrip)
{
    solver->m_angles_v.resize(3);
    for (int k = 0; k < 3; k++)
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            solver->m_angles_v[k].q[j] = 0.1 * k + 0.01 * j;
            solver->m_angles_v[k].q_p[j] = -0.2 * k + 0.02 * j;
            solver->m_angles_v[k].q_pp[j] = 0.3 * k - 0.03 * j;
        }
    std::vector<float> flat;
    solver->AnglesVectorToVector(3, flat);
    ASSERT_EQ(flat.size(), 3u * kAnglesFloats);
    for (int k = 0; k < 3; k++)
    {
        std::vector<float> one(flat.begin() + k * kAnglesFloats, flat.begin() + (k + 1) * kAnglesFloats);
        Angles a = solver->VectorToAngle(one);
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            EXPECT_NEAR(a.q[j], solver->m_angles_v[k].q[j], 1e-6);
            EXPECT_NEAR(a.q_p[j], solver->m_angles_v[k].q_p[j], 1e-6);
            EXPECT_NEAR(a.q_pp[j], solver->m_angles_v[k].q_pp[j], 1e-6);
        }
    }
}

TEST_F(SolverTest, PredefinedPositions)
{
    EXPECT_TRUE(solver->GetZeroPosition().IsNull());
    Angles in = solver->GetInternalPosition();
    Angles ex = solver->GetExtarnalPosition();
    const double in_expected[6] = {0, -M_PI / 4, M_PI / 2, M_PI / 4, M_PI / 2, 0};
    const double ex_expected[6] = {0, -M_PI / 4, M_PI / 2, -M_PI + M_PI / 3.7, -M_PI / 2, 0};
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_DOUBLE_EQ(in.q[j], in_expected[j]);
        EXPECT_DOUBLE_EQ(ex.q[j], ex_expected[j]);
        EXPECT_DOUBLE_EQ(in.q_p[j], 0);
        EXPECT_DOUBLE_EQ(ex.q_pp[j], 0);
    }
}

// NOTE (not tested): Solver::FindAngle is declared `inline` in solver.h but
// only defined in solver.cpp, so it cannot be linked from any other
// translation unit. BoxAABB's constructor and Solver::CheckAABB are declared
// but never defined at all.
