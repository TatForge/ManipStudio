// Phase 3: high-level trajectory behaviour of Solver::MoveJ / MoveL / MoveC.
//
// Each scenario starts from concrete initial conditions and checks the
// properties a user of the planner relies on: the move starts where it was
// asked to start, ends where it was asked to end (at rest), follows the
// requested path shape, respects the speed/acceleration limits, is smooth in
// joint space, and gives the same result every time it is called.
// Positions are verified with the independent reference FK (test_helpers.h).

#include "solver_fixture.h"

#include <algorithm>

using namespace solver_test;

namespace {

constexpr double kDtMs = 10;
constexpr double kDt = kDtMs / 1000.0;

struct Traj
{
    std::vector<Angles> samples;
    std::vector<Vector3> tcp; // reference FK of every sample
};

Traj MakeTraj(const std::vector<float> &flat)
{
    Traj t;
    t.samples = Samples(flat);
    for (const auto &a : t.samples)
        t.tcp.push_back(ReferenceTcp(a));
    return t;
}

double MaxJointStep(const Traj &t)
{
    double m = 0;
    for (size_t k = 1; k < t.samples.size(); k++)
        for (int j = 0; j < JOINTS_COUNT; j++)
            m = std::max(m, std::abs(t.samples[k].q[j] - t.samples[k - 1].q[j]));
    return m;
}

double MaxTcpSpeed(const Traj &t)
{
    double m = 0;
    for (size_t k = 1; k < t.tcp.size(); k++)
        m = std::max(m, Dist(t.tcp[k], t.tcp[k - 1]) / kDt);
    return m;
}

double MaxTcpAcceleration(const Traj &t)
{
    double m = 0;
    for (size_t k = 2; k < t.tcp.size(); k++)
    {
        Vector3 v1 = (t.tcp[k - 1] - t.tcp[k - 2]) / kDt;
        Vector3 v2 = (t.tcp[k] - t.tcp[k - 1]) / kDt;
        m = std::max(m, (v2 - v1).Length() / kDt);
    }
    return m;
}

// ---------------------------------------------------------------------------
// MoveJ
// ---------------------------------------------------------------------------

class MoveJTest : public SolverTest
{
protected:
    ::testing::AssertionResult Run(const Angles &from, const Angles &to, float v, float a, std::vector<float> &out,
                                   Solver *s = nullptr)
    {
        if (!s)
            s = solver.get();
        return RunsWithoutThrowing([&] { s->MoveJ(from, to, v, a, kDtMs, out); });
    }
};

// Scenario: only joint 0 moves 0.3 -> 0.8 rad, v = 1 rad/s, a = 2 rad/s^2.
class MoveJSingleJoint : public MoveJTest
{
protected:
    void SetUp() override
    {
        MoveJTest::SetUp();
        from = WorkPose();
        to = from;
        to.q[0] = 0.8;
        ASSERT_TRUE(Run(from, to, kV, kA, flat));
        traj = MakeTraj(flat);
        ASSERT_GT(traj.samples.size(), 2u);
    }
    static constexpr float kV = 1.0f;
    static constexpr float kA = 2.0f;
    Angles from, to;
    std::vector<float> flat;
    Traj traj;
};

TEST_F(MoveJSingleJoint, OutputIsWholeNumberOfSamples)
{
    EXPECT_EQ(flat.size() % kAnglesFloats, 0u);
}

TEST_F(MoveJSingleJoint, StartsAtStartAngles)
{
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_NEAR(traj.samples.front().q[j], from.q[j], 1e-6);
        EXPECT_NEAR(traj.samples.front().q_p[j], 0, 1e-6);
    }
}

// Regression test for a fixed bug: CalcTrajOneJoint writes the final (end) sample at
// index sizes[i], but MoveJ then truncates m_angles_v to max_size elements
// (indices 0..max_size-1), so the last sample of the slowest joint -- the
// one that actually reaches end_angle with zero speed -- is dropped. The
// trajectory stops ~0.3 mrad short, still moving.
TEST_F(MoveJSingleJoint, EndsExactlyAtEndAnglesAtRest)
{
    const Angles &last = traj.samples.back();
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_NEAR(last.q[j], to.q[j], 1e-6) << "joint " << j;
        EXPECT_NEAR(last.q_p[j], 0, 1e-6) << "joint " << j;
    }
}

TEST_F(MoveJSingleJoint, OtherJointsDoNotMove)
{
    for (const auto &s : traj.samples)
        for (int j = 1; j < JOINTS_COUNT; j++)
        {
            ASSERT_DOUBLE_EQ(static_cast<float>(s.q[j]), static_cast<float>(from.q[j])) << "joint " << j;
            ASSERT_DOUBLE_EQ(s.q_p[j], 0) << "joint " << j;
        }
}

TEST_F(MoveJSingleJoint, MovesMonotonicallyWithoutOvershoot)
{
    for (size_t k = 1; k < traj.samples.size(); k++)
    {
        EXPECT_GE(traj.samples[k].q[0], traj.samples[k - 1].q[0] - 1e-9) << "sample " << k;
        EXPECT_LE(traj.samples[k].q[0], to.q[0] + 1e-6) << "sample " << k;
    }
}

TEST_F(MoveJSingleJoint, RespectsSpeedLimit)
{
    for (size_t k = 0; k < traj.samples.size(); k++)
        EXPECT_LE(std::abs(traj.samples[k].q_p[0]), kV * 1.001) << "sample " << k;
    for (size_t k = 1; k < traj.samples.size(); k++)
        EXPECT_LE(std::abs(traj.samples[k].q[0] - traj.samples[k - 1].q[0]) / kDt, kV * 1.01) << "sample " << k;
}

// Regression test for a fixed bug: in braking mode CalcTrajOneJoint recomputes the
// deceleration every step from the remaining distance, which exceeds the
// requested limit (up to ~2.2 rad/s^2 for a 2 rad/s^2 limit).
TEST_F(MoveJSingleJoint, RespectsAccelerationLimit)
{
    for (size_t k = 0; k < traj.samples.size(); k++)
        EXPECT_LE(std::abs(traj.samples[k].q_pp[0]), kA * 1.01) << "sample " << k;
}

TEST_F(MoveJSingleJoint, ReportedSpeedIsConsistentWithPositions)
{
    for (size_t k = 1; k < traj.samples.size(); k++)
    {
        double from_positions = (traj.samples[k].q[0] - traj.samples[k - 1].q[0]) / kDt;
        double reported = 0.5 * (traj.samples[k].q_p[0] + traj.samples[k - 1].q_p[0]);
        EXPECT_NEAR(from_positions, reported, 0.05) << "sample " << k;
    }
}

// dq = 0.5 rad, v = 1, a = 2: speed-up distance v^2/(2a) = 0.25 rad = dq/2,
// so the profile is (just) triangular: T = 2*sqrt(dq/a) = 1 s.
TEST_F(MoveJSingleJoint, DurationMatchesTrapezoidalProfile)
{
    const double expected = 2 * std::sqrt(0.5 / kA);
    const double actual = (traj.samples.size() - 1) * kDt;
    EXPECT_NEAR(actual, expected, 0.1 * expected);
}

TEST_F(MoveJTest, NegativeDirectionMirrorsPositiveDirection)
{
    Angles a = WorkPose(), b = WorkPose();
    b.q[2] += 0.4;
    std::vector<float> up, down;
    ASSERT_TRUE(Run(a, b, 1.0f, 2.0f, up));
    auto s2 = NewSolver();
    ASSERT_TRUE(Run(b, a, 1.0f, 2.0f, down, s2.get()));
    Traj tu = MakeTraj(up), td = MakeTraj(down);
    ASSERT_EQ(tu.samples.size(), td.samples.size());
    for (size_t k = 0; k < tu.samples.size(); k++)
        EXPECT_NEAR(tu.samples[k].q[2] - a.q[2], b.q[2] - td.samples[k].q[2], 1e-6) << "sample " << k;
}

// Scenario: zero pose -> GetInternalPosition() (four joints move by different
// amounts). Every joint must reach its own target and then hold still.
TEST_F(MoveJTest, MultiJointMoveReachesEveryTargetAndHolds)
{
    Angles from = solver->GetZeroPosition(), to = solver->GetInternalPosition();
    std::vector<float> flat;
    ASSERT_TRUE(Run(from, to, 1.0f, 2.0f, flat));
    Traj t = MakeTraj(flat);
    ASSERT_GT(t.samples.size(), 2u);
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_NEAR(t.samples.back().q[j], to.q[j], 1e-6) << "joint " << j;
        // once a joint reached its target it must stay there
        size_t reached = t.samples.size();
        for (size_t k = 0; k < t.samples.size(); k++)
            if (std::abs(t.samples[k].q[j] - to.q[j]) < 1e-9)
            {
                reached = k;
                break;
            }
        for (size_t k = reached; k < t.samples.size(); k++)
        {
            EXPECT_NEAR(t.samples[k].q[j], to.q[j], 1e-9) << "joint " << j << " sample " << k;
            EXPECT_NEAR(t.samples[k].q_p[j], 0, 1e-9) << "joint " << j << " sample " << k;
        }
    }
}

// Synchronous PTP: all joints start and stop together and every sample lies
// on the straight joint-space line between start and end; the joint with
// the largest travel runs the time-optimal profile.
TEST_F(MoveJTest, JointsMoveSynchronously)
{
    Angles from = WorkPose(), to = WorkPose();
    to.q[0] += 0.8; // largest travel -> leads
    to.q[1] -= 0.2;
    to.q[3] += 0.5;
    to.q[5] -= 0.05;
    std::vector<float> flat;
    ASSERT_TRUE(Run(from, to, 1.0f, 2.0f, flat));
    auto s = Samples(flat);
    ASSERT_GT(s.size(), 2u);
    for (size_t k = 0; k < s.size(); k++)
    {
        const double u = (s[k].q[0] - from.q[0]) / (to.q[0] - from.q[0]);
        for (int j = 1; j < JOINTS_COUNT; j++)
        {
            const double delta = to.q[j] - from.q[j];
            EXPECT_NEAR(s[k].q[j], from.q[j] + delta * u, 1e-6) << "joint " << j << " sample " << k;
            EXPECT_LE(std::abs(s[k].q_p[j]), 1.0 * 1.001) << "joint " << j << " sample " << k;
        }
    }
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_NEAR(s.back().q[j], to.q[j], 1e-6) << "joint " << j;
        EXPECT_NEAR(s.back().q_p[j], 0, 1e-9) << "joint " << j;
        // nobody arrives early
        EXPECT_GT(std::abs(s[s.size() - 2].q[j] - to.q[j]) + (to.q[j] == from.q[j]), 0) << "joint " << j;
    }
    // duration of the leading joint alone: 0.8 rad at v = 1, a = 2 -> 1.3 s
    EXPECT_NEAR((s.size() - 1) * kDt, 0.8 / 1.0 + 1.0 / 2.0, 0.02);
}

TEST_F(MoveJTest, ThrowsWhenStartEqualsEnd)
{
    std::vector<float> flat;
    ScopedSilence quiet;
    EXPECT_THROW(solver->MoveJ(WorkPose(), WorkPose(), 1.0f, 2.0f, kDtMs, flat), std::runtime_error);
}

// Planning the same move twice must give the same trajectory.
TEST_F(MoveJTest, IsRepeatableOnSameSolver)
{
    Angles from = WorkPose(), to = WorkPose();
    to.q[1] -= 0.3;
    to.q[3] += 0.6;
    std::vector<float> first, second;
    ASSERT_TRUE(Run(from, to, 1.0f, 2.0f, first));
    ASSERT_TRUE(Run(from, to, 1.0f, 2.0f, second));
    ASSERT_EQ(first.size(), second.size());
    for (size_t i = 0; i < first.size(); i++)
        ASSERT_FLOAT_EQ(first[i], second[i]) << "index " << i;
}

// A planner call must not depend on what was planned before. Here a long
// move leaves data in the Solver's internal buffers; a following short move
// must be identical to the same short move on a fresh Solver. MoveJ resizes
// m_angles_v without clearing it, so q_p/q_pp of joints that finished early
// keep stale values from the previous call.
TEST_F(MoveJTest, DoesNotDependOnPreviousCall)
{
    Angles from = WorkPose(), far_to = WorkPose(), near_to = WorkPose();
    far_to.q[1] -= 0.8;
    far_to.q[4] += 1.0;
    near_to.q[0] += 0.2;
    near_to.q[4] += 0.1;
    std::vector<float> warmup, after, fresh;
    ASSERT_TRUE(Run(from, far_to, 1.0f, 2.0f, warmup));
    ASSERT_TRUE(Run(from, near_to, 1.0f, 2.0f, after));
    auto s2 = NewSolver();
    ASSERT_TRUE(Run(from, near_to, 1.0f, 2.0f, fresh, s2.get()));
    ASSERT_EQ(after.size(), fresh.size());
    for (size_t i = 0; i < after.size(); i++)
        ASSERT_FLOAT_EQ(after[i], fresh[i]) << "index " << i;
}

// ---------------------------------------------------------------------------
// MoveL
// ---------------------------------------------------------------------------

class MoveLTest : public SolverTest
{
protected:
    ::testing::AssertionResult Run(const StateVector &p1, const StateVector &p2, double v, double a, const Angles &start,
                                   std::vector<float> &out, Solver *s = nullptr)
    {
        if (!s)
            s = solver.get();
        return RunsWithoutThrowing([&] { s->MoveL(p1, p2, v, a, start, kDtMs, out); });
    }
};

// Scenario: from WorkPose() move the TCP 10 cm towards the base (-X) and
// 5 cm down, keeping the orientation; v = 0.1 m/s, a = 0.5 m/s^2.
class MoveLStraight : public MoveLTest
{
protected:
    void SetUp() override
    {
        MoveLTest::SetUp();
        start = WorkPose();
        p1 = ReferencePose(start);
        p2 = p1;
        p2.x -= 0.10;
        p2.z -= 0.05;
        ASSERT_TRUE(Run(p1, p2, kV, kA, start, flat));
        traj = MakeTraj(flat);
        ASSERT_GT(traj.samples.size(), 2u);
    }
    static constexpr double kV = 0.1;
    static constexpr double kA = 0.5;
    Angles start;
    StateVector p1, p2;
    std::vector<float> flat;
    Traj traj;
};

TEST_F(MoveLStraight, StartsAtStartPoint)
{
    EXPECT_VEC3_NEAR(traj.tcp.front(), p1.GetCoord(), 1e-6);
    for (int j = 0; j < JOINTS_COUNT; j++)
        EXPECT_NEAR(traj.samples.front().q[j], start.q[j], 1e-5) << "joint " << j;
}

// Regression test for a fixed bug (same pattern as MoveJ): the last computed sample
// m_angles_v[size] -- the one placed exactly at end_point -- is not copied
// to the output (AnglesVectorToVector(size, ...) writes indices 0..size-1).
TEST_F(MoveLStraight, EndsAtEndPoint)
{
    EXPECT_VEC3_NEAR(traj.tcp.back(), p2.GetCoord(), 1e-5);
}

TEST_F(MoveLStraight, EveryPointLiesOnTheLine)
{
    Vector3 a = p1.GetCoord(), b = p2.GetCoord();
    for (size_t k = 0; k < traj.tcp.size(); k++)
        EXPECT_LE(CalcDistancePointToLine(traj.tcp[k], &a, &b), DELTA_RANGE) << "sample " << k;
}

TEST_F(MoveLStraight, ProgressesMonotonicallyTowardsEnd)
{
    Vector3 dir = (p2.GetCoord() - p1.GetCoord()).Normalize();
    double prev = -1e-9;
    for (size_t k = 0; k < traj.tcp.size(); k++)
    {
        double s = (traj.tcp[k] - p1.GetCoord()).Dot(dir);
        EXPECT_GE(s, prev - 1e-7) << "sample " << k;
        EXPECT_LE(s, (p2.GetCoord() - p1.GetCoord()).Length() + 1e-6) << "sample " << k;
        prev = s;
    }
}

TEST_F(MoveLStraight, KeepsToolOrientation)
{
    Mat4 ref = ReferenceTcpFrame(start);
    for (size_t k = 0; k < traj.samples.size(); k++)
        EXPECT_LT(RotationDistance(ReferenceTcpFrame(traj.samples[k]), ref), 1e-4) << "sample " << k;
}

TEST_F(MoveLStraight, RespectsTcpSpeedLimit)
{
    EXPECT_LE(MaxTcpSpeed(traj), kV * 1.02);
}

TEST_F(MoveLStraight, RespectsTcpAccelerationLimit)
{
    EXPECT_LE(MaxTcpAcceleration(traj), kA * 1.1);
}

// distance 0.1118 m, v = 0.1, a = 0.5: 0.2 s speed-up, 0.2 s braking,
// 0.918 s cruise -> 1.32 s.
TEST_F(MoveLStraight, DurationMatchesTrapezoidalProfile)
{
    const double d = (p2.GetCoord() - p1.GetCoord()).Length();
    const double expected = d / kV + kV / kA;
    EXPECT_NEAR((traj.samples.size() - 1) * kDt, expected, 0.1 * expected);
}

TEST_F(MoveLStraight, JointMotionIsSmooth)
{
    EXPECT_LT(MaxJointStep(traj), 0.02); // < 2 rad/s at 10 ms
    for (size_t k = 0; k < traj.samples.size(); k++)
        for (int j = 0; j < JOINTS_COUNT; j++)
            EXPECT_LT(std::abs(traj.samples[k].q_pp[j]), 20.0) << "sample " << k << " joint " << j;
}

TEST_F(MoveLStraight, IsRepeatableOnSameSolver)
{
    std::vector<float> again;
    ASSERT_TRUE(Run(p1, p2, kV, kA, start, again));
    ASSERT_EQ(again.size(), flat.size());
    for (size_t i = 0; i < flat.size(); i++)
        ASSERT_FLOAT_EQ(again[i], flat[i]) << "index " << i;
}

TEST_F(MoveLStraight, DoesNotDependOnPreviousMoveJ)
{
    auto s2 = NewSolver();
    std::vector<float> tmp, after;
    Angles other = WorkPose();
    other.q[0] -= 0.5;
    ASSERT_TRUE(RunsWithoutThrowing([&] { s2->MoveJ(other, WorkPose(), 1.0f, 2.0f, kDtMs, tmp); }));
    ASSERT_TRUE(Run(p1, p2, kV, kA, start, after, s2.get()));
    ASSERT_EQ(after.size(), flat.size());
    for (size_t i = 0; i < flat.size(); i++)
        ASSERT_FLOAT_EQ(after[i], flat[i]) << "index " << i;
}

// Scenario: change the tool orientation (yaw +0.3 rad) while moving 5 cm
// along +Y. Orientation must be interpolated and reach the target.
TEST_F(MoveLTest, InterpolatesOrientationAndReachesTarget)
{
    Angles start = WorkPose();
    StateVector p1 = ReferencePose(start), p2 = p1;
    p2.y += 0.05;
    p2.yaw += 0.3;
    std::vector<float> flat;
    ASSERT_TRUE(Run(p1, p2, 0.1, 0.5, start, flat));
    Traj t = MakeTraj(flat);
    ASSERT_GT(t.samples.size(), 2u);
    Mat4 target = EulerMatrix(p2.GetAngles());
    double prev = RotationDistance(ReferenceTcpFrame(t.samples.front()), target);
    for (size_t k = 1; k < t.samples.size(); k++)
    {
        double d = RotationDistance(ReferenceTcpFrame(t.samples[k]), target);
        EXPECT_LE(d, prev + 1e-6) << "sample " << k;
        prev = d;
    }
    EXPECT_LT(prev, 1e-3);
}

// Scenario: the documented start pose GetInternalPosition(), 5 cm along +X.
// Used to fail because of the wrong start orientation at this pose (see
// SolverTest.ReportedOrientationOfPredefinedPosesIsCorrect): the first IK
// step jumps to another arm configuration (joint 3 moves ~5 rad in 10 ms)
// and the dynamics check throws with a ~1600 Nm moment on joint 0.
TEST_F(MoveLTest, WorksFromInternalPosition)
{
    Angles start = solver->GetInternalPosition();
    StateVector p1 = SolverPose(start), p2 = p1;
    p2.x += 0.05;
    std::vector<float> flat;
    EXPECT_TRUE(Run(p1, p2, 0.1, 0.5, start, flat));
}

// A line that leaves the workspace (arm would have to stretch beyond full
// extension) must be rejected with an exception, not produce a trajectory.
TEST_F(MoveLTest, RejectsLineLeavingWorkspace)
{
    Angles start = WorkPose();
    StateVector p1 = ReferencePose(start), p2 = p1;
    p2.x += 0.3;
    std::vector<float> flat;
    ScopedSilence quiet;
    EXPECT_ANY_THROW(solver->MoveL(p1, p2, 0.1, 0.5, start, kDtMs, flat));
}

// ---------------------------------------------------------------------------
// MoveC
// ---------------------------------------------------------------------------

struct Circle
{
    Vector3 center;
    Vector3 normal;
    double radius;
};

Circle Circumcircle(const Vector3 &a, const Vector3 &b, const Vector3 &c)
{
    Vector3 ab = b - a, ac = c - a;
    Vector3 n = ab * ac;
    double n2 = n.Dot(n);
    Vector3 to_center = ((n * ab) * ac.Dot(ac) + (ac * n) * ab.Dot(ab)) / (2 * n2);
    return {a + to_center, n.Normalize(), to_center.Length()};
}

class MoveCTest : public SolverTest
{
protected:
    ::testing::AssertionResult Run(const Angles &start, const StateVector &a, const StateVector &b, const StateVector &c,
                                   double v, double acc, std::vector<float> &out, Solver *s = nullptr)
    {
        if (!s)
            s = solver.get();
        return RunsWithoutThrowing([&] { s->MoveC(start, a, b, c, v, acc, kDtMs, out); });
    }
};

// Scenario: from WorkPose() a half circle of radius 5 cm in the vertical XZ
// plane: A = TCP, B = A + (-5, 0, -5) cm, C = A + (-10, 0, 0) cm.
class MoveCArc : public MoveCTest
{
protected:
    void SetUp() override
    {
        MoveCTest::SetUp();
        start = WorkPose();
        a = ReferencePose(start);
        b = a;
        b.x -= 0.05;
        b.z -= 0.05;
        c = a;
        c.x -= 0.10;
        circle = Circumcircle(a.GetCoord(), b.GetCoord(), c.GetCoord());
        ASSERT_TRUE(Run(start, a, b, c, kV, kA, flat));
        traj = MakeTraj(flat);
        ASSERT_GT(traj.samples.size(), 2u);
    }
    static constexpr double kV = 0.1;
    static constexpr double kA = 0.5;
    Angles start;
    StateVector a, b, c;
    Circle circle;
    std::vector<float> flat;
    Traj traj;
};

TEST_F(MoveCArc, ReferenceCircleIsAsDesigned)
{
    EXPECT_NEAR(circle.radius, 0.05, 1e-12);
    EXPECT_VEC3_NEAR(circle.center, a.GetCoord() + Vector3(-0.05, 0, 0), 1e-12);
}

TEST_F(MoveCArc, StartsAtFirstPoint)
{
    EXPECT_VEC3_NEAR(traj.tcp.front(), a.GetCoord(), 1e-6);
}

TEST_F(MoveCArc, EndsAtLastPoint)
{
    EXPECT_VEC3_NEAR(traj.tcp.back(), c.GetCoord(), 1e-4);
}

TEST_F(MoveCArc, EveryPointLiesOnTheCircle)
{
    for (size_t k = 0; k < traj.tcp.size(); k++)
    {
        Vector3 r = traj.tcp[k] - circle.center;
        EXPECT_NEAR(r.Length(), circle.radius, 1e-4) << "sample " << k;
        EXPECT_NEAR(r.Dot(circle.normal), 0, 1e-4) << "sample " << k;
    }
}

TEST_F(MoveCArc, PassesThroughMiddlePoint)
{
    double closest = 1e9;
    for (const auto &p : traj.tcp)
        closest = std::min(closest, Dist(p, b.GetCoord()));
    EXPECT_LT(closest, kV * kDt); // within one step
}

// The arc must run A -> B -> C (not the complementary arc through the other
// side of the circle): the swept angle must be the 180 deg via B.
TEST_F(MoveCArc, TravelsArcThroughMiddlePointMonotonically)
{
    Vector3 u = (a.GetCoord() - circle.center).Normalize();
    Vector3 w = circle.normal * u;
    auto angle_of = [&](const Vector3 &p) {
        Vector3 r = p - circle.center;
        double ang = atan2(r.Dot(w), r.Dot(u));
        return ang;
    };
    const double b_angle = angle_of(b.GetCoord());
    const double dir = b_angle > 0 ? 1 : -1;
    double prev = 0;
    for (size_t k = 1; k < traj.tcp.size(); k++)
    {
        double ang = dir * angle_of(traj.tcp[k]);
        if (ang < -M_PI / 2)
            ang += 2 * M_PI; // unwrap near the end (C is at +-180 deg)
        EXPECT_GE(ang, prev - 1e-6) << "sample " << k;
        prev = ang;
    }
    EXPECT_NEAR(prev, M_PI, 0.05);
}

TEST_F(MoveCArc, RespectsTcpSpeedLimit)
{
    EXPECT_LE(MaxTcpSpeed(traj), kV * 1.02);
}

TEST_F(MoveCArc, KeepsToolOrientation)
{
    Mat4 ref = ReferenceTcpFrame(start);
    for (size_t k = 0; k < traj.samples.size(); k++)
        EXPECT_LT(RotationDistance(ReferenceTcpFrame(traj.samples[k]), ref), 1e-3) << "sample " << k;
}

TEST_F(MoveCArc, JointMotionIsSmooth)
{
    EXPECT_LT(MaxJointStep(traj), 0.02);
}

// Arc length pi * 5 cm = 0.157 m at v = 0.1, a = 0.5 -> ~1.77 s.
TEST_F(MoveCArc, DurationMatchesTrapezoidalProfile)
{
    const double expected = M_PI * circle.radius / kV + kV / kA;
    EXPECT_NEAR((traj.samples.size() - 1) * kDt, expected, 0.1 * expected);
}

TEST_F(MoveCTest, ThrowsForCollinearPoints)
{
    StateVector a = ReferencePose(WorkPose()), b = a, c = a;
    b.x -= 0.05;
    c.x -= 0.10;
    std::vector<float> flat;
    ScopedSilence quiet;
    EXPECT_THROW(solver->MoveC(WorkPose(), a, b, c, 0.1, 0.5, kDtMs, flat), std::runtime_error);
}

// Scenario: a quarter circle in the horizontal plane (A, B at 45 deg, C at
// 90 deg), radius 4 cm -- checks MoveC in another plane and for an arc that
// is not a half circle.
TEST_F(MoveCTest, QuarterArcInHorizontalPlane)
{
    Angles start = WorkPose();
    StateVector a = ReferencePose(start), b = a, c = a;
    const double r = 0.04;
    // centre at A + (0, -r, 0)
    b.x += r * sin(M_PI / 4);
    b.y += -r + r * cos(M_PI / 4);
    c.x += r;
    c.y -= r;
    std::vector<float> flat;
    ASSERT_TRUE(Run(start, a, b, c, 0.1, 0.5, flat));
    Traj t = MakeTraj(flat);
    ASSERT_GT(t.samples.size(), 2u);
    Vector3 center = a.GetCoord() + Vector3(0, -r, 0);
    for (size_t k = 0; k < t.tcp.size(); k++)
    {
        EXPECT_NEAR(Dist(t.tcp[k], center), r, 1e-4) << "sample " << k;
        EXPECT_NEAR(t.tcp[k].z, a.z, 1e-4) << "sample " << k;
    }
    EXPECT_VEC3_NEAR(t.tcp.back(), c.GetCoord(), 1e-4);
}

TEST_F(MoveCArc, IsRepeatableOnSameSolver)
{
    std::vector<float> again;
    ASSERT_TRUE(Run(start, a, b, c, kV, kA, again));
    ASSERT_EQ(again.size(), flat.size());
    for (size_t i = 0; i < flat.size(); i++)
        ASSERT_FLOAT_EQ(again[i], flat[i]) << "index " << i;
}

// ===========================================================================
// Regression net: parameter sweeps, long moves and edge cases.
// Tolerances here accept the small, separately documented defects (last
// sample missing, +2.5 % cruise speed) so these tests stay green before and
// after the fixes -- they guard against *new* breakage.
// ===========================================================================

// ---------------------------------------------------------------------------
// MoveJ parameter sweep
// ---------------------------------------------------------------------------

struct MoveJCase
{
    int joint;
    double delta; // rad
    float v;      // rad/s
    float a;      // rad/s^2
    float dt_ms;
};

std::ostream &operator<<(std::ostream &os, const MoveJCase &c)
{
    return os << "joint " << c.joint << " delta " << c.delta << " v " << c.v << " a " << c.a << " dt " << c.dt_ms;
}

class MoveJSweep : public MoveJTest, public ::testing::WithParamInterface<MoveJCase>
{
};

TEST_P(MoveJSweep, ProfileIsCorrect)
{
    const MoveJCase c = GetParam();
    const double dt = c.dt_ms / 1000.0;
    Angles from = WorkPose(), to = WorkPose();
    to.q[c.joint] += c.delta;
    std::vector<float> flat;
    ASSERT_TRUE(RunsWithoutThrowing([&] { solver->MoveJ(from, to, c.v, c.a, c.dt_ms, flat); }));
    auto s = Samples(flat);
    ASSERT_GT(s.size(), 1u);
    const double dir = c.delta > 0 ? 1 : -1;
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_NEAR(s.front().q[j], from.q[j], 1e-6) << "joint " << j;
        // known defect: the final sample is dropped (MoveJSingleJoint.EndsExactlyAtEndAnglesAtRest)
        EXPECT_NEAR(s.back().q[j], to.q[j], 1e-3 + 0.1 * c.v * dt) << "joint " << j;
    }
    for (size_t k = 1; k < s.size(); k++)
    {
        EXPECT_GE(dir * (s[k].q[c.joint] - s[k - 1].q[c.joint]), -1e-9) << "sample " << k;
        EXPECT_LE(std::abs(s[k].q_p[c.joint]), c.v * 1.001) << "sample " << k;
        // known defect: speed is updated before position, so a step may exceed v*dt by a*dt^2/2
        EXPECT_LE(std::abs(s[k].q[c.joint] - s[k - 1].q[c.joint]), c.v * dt * 1.01 + 0.5 * c.a * dt * dt + 1e-6) << "sample " << k;
        for (int j = 0; j < JOINTS_COUNT; j++)
            if (j != c.joint)
                ASSERT_FLOAT_EQ(float(s[k].q[j]), float(from.q[j])) << "joint " << j << " sample " << k;
    }
    const double d = std::abs(c.delta);
    const double expected = d >= c.v * c.v / c.a ? d / c.v + c.v / c.a : 2 * std::sqrt(d / c.a);
    EXPECT_NEAR((s.size() - 1) * dt, expected, 0.12 * expected + 2 * dt);
}

INSTANTIATE_TEST_SUITE_P(Profiles, MoveJSweep,
                         ::testing::Values(MoveJCase{0, 0.5, 1.0f, 2.0f, 10},   // triangular (limit)
                                           MoveJCase{1, 0.4, 0.5f, 1.0f, 10},   // trapezoidal
                                           MoveJCase{2, -0.6, 0.5f, 1.0f, 10},  // negative direction
                                           MoveJCase{3, 1.2, 2.0f, 4.0f, 10},   // fast wrist
                                           MoveJCase{4, 0.3, 0.3f, 3.0f, 10},   // long cruise
                                           MoveJCase{5, -1.0, 1.5f, 1.0f, 10},  // slow acceleration
                                           MoveJCase{1, 0.05, 1.0f, 2.0f, 10},  // tiny move
                                           MoveJCase{0, 0.5, 1.0f, 2.0f, 5},    // finer time step
                                           MoveJCase{0, 0.5, 1.0f, 2.0f, 20})); // coarser time step

// More than 500 samples: exercises the VectorsResize() path.
TEST_F(MoveJTest, LongMoveBeyondInitialBufferIsContinuous)
{
    Angles from = WorkPose(), to = WorkPose();
    to.q[0] += 2.5; // 5.5 s at 0.5 rad/s
    std::vector<float> flat;
    ASSERT_TRUE(Run(from, to, 0.5f, 1.0f, flat));
    auto s = Samples(flat);
    ASSERT_GT(s.size(), 520u);
    for (size_t k = 1; k < s.size(); k++)
    {
        double step = s[k].q[0] - s[k - 1].q[0];
        ASSERT_GE(step, -1e-9) << "sample " << k;
        ASSERT_LE(step, 0.5 * kDt * 1.01 + 1e-6) << "sample " << k;
    }
    EXPECT_NEAR(s[s.size() / 2].q_p[0], 0.5, 1e-6); // cruising in the middle
    EXPECT_NEAR(s.back().q[0], to.q[0], 1e-3);
}

// ---------------------------------------------------------------------------
// MoveL direction / time-step sweep
// ---------------------------------------------------------------------------

struct MoveLCase
{
    Vector3 offset; // m
    float dt_ms;
};

std::ostream &operator<<(std::ostream &os, const MoveLCase &c)
{
    return os << "offset (" << c.offset.x << ", " << c.offset.y << ", " << c.offset.z << ") dt " << c.dt_ms;
}

class MoveLSweep : public MoveLTest, public ::testing::WithParamInterface<MoveLCase>
{
};

TEST_P(MoveLSweep, FollowsLineAndReachesEnd)
{
    const MoveLCase c = GetParam();
    const double dt = c.dt_ms / 1000.0, v = 0.1;
    Angles start = WorkPose();
    StateVector p1 = ReferencePose(start), p2 = p1;
    p2.x += c.offset.x;
    p2.y += c.offset.y;
    p2.z += c.offset.z;
    std::vector<float> flat;
    ASSERT_TRUE(RunsWithoutThrowing([&] { solver->MoveL(p1, p2, v, 0.5, start, ushort(c.dt_ms), flat); }));
    Traj t = MakeTraj(flat);
    ASSERT_GT(t.samples.size(), 2u);
    Vector3 a = p1.GetCoord(), b = p2.GetCoord();
    Mat4 ori = ReferenceTcpFrame(start);
    for (size_t k = 0; k < t.tcp.size(); k++)
    {
        EXPECT_LE(CalcDistancePointToLine(t.tcp[k], &a, &b), DELTA_RANGE) << "sample " << k;
        EXPECT_LT(RotationDistance(ReferenceTcpFrame(t.samples[k]), ori), 1e-4) << "sample " << k;
        // known defect: +a*dt/2 during cruise (MoveLStraight.RespectsTcpSpeedLimit)
        if (k > 0)
            EXPECT_LE(Dist(t.tcp[k], t.tcp[k - 1]) / dt, v * 1.01 + 0.5 * 0.5 * dt) << "sample " << k;
    }
    EXPECT_VEC3_NEAR(t.tcp.front(), a, 1e-6);
    // known defect: the final sample is dropped (MoveLStraight.EndsAtEndPoint)
    EXPECT_VEC3_NEAR(t.tcp.back(), b, 1e-4 + 0.1 * v * dt);
    EXPECT_LT(MaxJointStep(t), 2.0 * dt);
}

INSTANTIATE_TEST_SUITE_P(Directions, MoveLSweep,
                         ::testing::Values(MoveLCase{Vector3(0.05, 0, 0), 10}, MoveLCase{Vector3(-0.05, 0, 0), 10},
                                           MoveLCase{Vector3(0, 0.05, 0), 10}, MoveLCase{Vector3(0, -0.05, 0), 10},
                                           MoveLCase{Vector3(0, 0, 0.05), 10}, MoveLCase{Vector3(0, 0, -0.05), 10},
                                           MoveLCase{Vector3(-0.03, 0.03, -0.03), 10},
                                           MoveLCase{Vector3(-0.05, 0, -0.02), 5},
                                           MoveLCase{Vector3(-0.05, 0, -0.02), 20}));

// More than 500 samples: exercises VectorsResize() inside MoveL.
TEST_F(MoveLTest, LongSlowLineBeyondInitialBuffer)
{
    Angles start = WorkPose();
    StateVector p1 = ReferencePose(start), p2 = p1;
    p2.x -= 0.2; // 0.2 m at 0.03 m/s -> ~6.8 s
    std::vector<float> flat;
    ASSERT_TRUE(Run(p1, p2, 0.03, 0.3, start, flat));
    Traj t = MakeTraj(flat);
    ASSERT_GT(t.samples.size(), 520u);
    Vector3 a = p1.GetCoord(), b = p2.GetCoord();
    for (size_t k = 0; k < t.tcp.size(); k++)
        EXPECT_LE(CalcDistancePointToLine(t.tcp[k], &a, &b), DELTA_RANGE) << "sample " << k;
    EXPECT_VEC3_NEAR(t.tcp.back(), b, 1e-4);
    EXPECT_LT(MaxJointStep(t), 0.02);
}

// The given joint state need not be exactly at start_point: the planner
// first solves IK for start_point, and the trajectory starts there.
TEST_F(MoveLTest, StartsAtStartPointEvenIfCurrentAnglesDiffer)
{
    Angles exact = WorkPose(), nearby = WorkPose();
    nearby.q[0] += 0.02;
    nearby.q[2] -= 0.03;
    StateVector p1 = ReferencePose(exact), p2 = p1;
    p2.z -= 0.05;
    std::vector<float> flat;
    ASSERT_TRUE(Run(p1, p2, 0.1, 0.5, nearby, flat));
    Traj t = MakeTraj(flat);
    ASSERT_GT(t.samples.size(), 2u);
    EXPECT_VEC3_NEAR(t.tcp.front(), p1.GetCoord(), 1e-6);
}

// Rotating the tool in place (same position, new orientation) is a valid
// linear move. Today distance == 0 gives 0/0 direction vectors, the loop
// never runs and the code then indexes m_angles_v[size - 1] with size == 0.
// Run in a child process because it may crash.
TEST_F(MoveLTest, RotationInPlaceWorks)
{
    Angles start = WorkPose();
    StateVector p1 = ReferencePose(start), p2 = p1;
    p2.yaw += 0.3;
    EXPECT_EXIT(
        {
            std::vector<float> flat;
            try
            {
                ScopedSilence quiet;
                solver->MoveL(p1, p2, 0.1, 0.5, start, 10, flat);
            }
            catch (...)
            {
                std::exit(2);
            }
            auto s = Samples(flat);
            if (s.empty())
                std::exit(3);
            bool ok = Dist(ReferenceTcp(s.back()), p1.GetCoord()) < 1e-4 &&
                      RotationDistance(ReferenceTcpFrame(s.back()), EulerMatrix(p2.GetAngles())) < 1e-3;
            std::exit(ok ? 0 : 4);
        },
        ::testing::ExitedWithCode(0), "");
}

// ---------------------------------------------------------------------------
// MoveC edge cases
// ---------------------------------------------------------------------------

namespace {

// Signed angle of p around `center` in the plane (u, w), in [0, 2*pi).
double PlaneAngle(const Vector3 &p, const Vector3 &center, const Vector3 &u, const Vector3 &w)
{
    Vector3 r = p - center;
    double a = atan2(r.Dot(w), r.Dot(u));
    return a < -1e-9 ? a + 2 * M_PI : a;
}

} // namespace

// Scenario: 270 deg arc in the horizontal plane, radius 4 cm: A at 0 deg,
// B at 135 deg, C at 270 deg. The arc through B is the long way round.
TEST_F(MoveCTest, ArcLongerThanHalfCircleGoesThroughMiddlePoint)
{
    Angles start = WorkPose();
    const double r = 0.04;
    StateVector a = ReferencePose(start);
    Vector3 u(1, 0, 0), w(0, 1, 0);
    Vector3 center = a.GetCoord() - u * r;
    auto at = [&](double deg) {
        StateVector p = a;
        Vector3 c = center + u * (r * cos(ToRad(deg))) + w * (r * sin(ToRad(deg)));
        p.x = c.x;
        p.y = c.y;
        p.z = c.z;
        return p;
    };
    StateVector b = at(135), c = at(270);
    std::vector<float> flat;
    ASSERT_TRUE(Run(start, a, b, c, 0.1, 0.5, flat));
    Traj t = MakeTraj(flat);
    ASSERT_GT(t.samples.size(), 2u);
    double prev = 0;
    for (size_t k = 0; k < t.tcp.size(); k++)
    {
        EXPECT_NEAR(Dist(t.tcp[k], center), r, 1e-4) << "sample " << k;
        double ang = PlaneAngle(t.tcp[k], center, u, w);
        if (k > 0)
            EXPECT_GE(ang, prev - 1e-6) << "sample " << k;
        prev = ang;
    }
    EXPECT_NEAR(prev, 3 * M_PI / 2, 0.02);
    EXPECT_VEC3_NEAR(t.tcp.back(), c.GetCoord(), 1e-3);
}

// Same circle as MoveCArc, travelled backwards (C -> B -> A).
TEST_F(MoveCArc, ReverseDirectionFollowsSameCircle)
{
    Angles start_c;
    ASSERT_TRUE(RunsWithoutThrowing([&] { solver->SolveInverseKinematic(c, start, start_c); }));
    auto s2 = NewSolver();
    std::vector<float> back;
    ASSERT_TRUE(Run(start_c, c, b, a, kV, kA, back, s2.get()));
    Traj t = MakeTraj(back);
    ASSERT_GT(t.samples.size(), 2u);
    for (size_t k = 0; k < t.tcp.size(); k++)
        EXPECT_NEAR(Dist(t.tcp[k], circle.center), circle.radius, 1e-4) << "sample " << k;
    double closest = 1e9;
    for (const auto &p : t.tcp)
        closest = std::min(closest, Dist(p, b.GetCoord()));
    EXPECT_LT(closest, kV * kDt);
    EXPECT_VEC3_NEAR(t.tcp.back(), a.GetCoord(), 1e-3);
}

// More than 500 samples (half circle, r = 5 cm, at 2 cm/s -> ~790 samples).
// MoveC grows m_angles_v and `points` itself but never m_moments_v, which
// CalcCollisionsDynamics then writes past its end. Child process: heap
// corruption may crash.
TEST_F(MoveCArc, SlowArcBeyondInitialBuffer)
{
    EXPECT_EXIT(
        {
            auto s2 = NewSolver();
            std::vector<float> flat;
            try
            {
                ScopedSilence quiet;
                s2->MoveC(start, a, b, c, 0.02, 0.2, kDtMs, flat);
            }
            catch (...)
            {
                std::exit(2);
            }
            Traj t = MakeTraj(flat);
            if (t.samples.size() < 600)
                std::exit(3);
            for (const auto &p : t.tcp)
                if (std::abs(Dist(p, circle.center) - circle.radius) > 1e-4)
                    std::exit(4);
            s2.reset();
            std::exit(0);
        },
        ::testing::ExitedWithCode(0), "");
}

// Regression test for a removed limitation: MoveC throws "Cant solve MoveC task with 1000
// iterations" for any arc that takes longer than 1000 steps (10 s at 10 ms),
// e.g. this 16 s half circle -- MoveL/MoveJ have no such limit.
TEST_F(MoveCArc, VerySlowArcIsNotLimitedTo1000Steps)
{
    EXPECT_EXIT(
        {
            auto s2 = NewSolver();
            std::vector<float> flat;
            try
            {
                ScopedSilence quiet;
                s2->MoveC(start, a, b, c, 0.01, 0.1, kDtMs, flat);
            }
            catch (...)
            {
                std::exit(2);
            }
            std::exit(Samples(flat).size() > 1200 ? 0 : 3);
        },
        ::testing::ExitedWithCode(0), "");
}

} // namespace
