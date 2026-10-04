// Joint speed and acceleration limits (Robo::SetMaxJointSpeed / SetMaxJointAcceleration).
//
// MoveJ caps the requested speed / acceleration so that no joint exceeds its
// own limit while all joints still start and stop together. MoveL / MoveC keep
// the Cartesian path and slow the whole move down uniformly when a joint would
// exceed a limit; Solver::GetLastSpeedLimit() tells by how much and why.

#include "solver_fixture.h"

#include <algorithm>

using namespace solver_test;

namespace {

constexpr double kDtMs = 10;
constexpr double kDt = kDtMs / 1000.0;

struct Peaks
{
    std::array<double, JOINTS_COUNT> speed{};
    std::array<double, JOINTS_COUNT> accel{};
};

Peaks PeaksOf(const std::vector<Angles> &s)
{
    Peaks p;
    for (const Angles &a : s)
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            p.speed[j] = std::max(p.speed[j], std::abs(double(a.q_p[j])));
            p.accel[j] = std::max(p.accel[j], std::abs(double(a.q_pp[j])));
        }
    return p;
}

// joint speed measured from the positions alone, independent of q_p
double MeasuredSpeed(const std::vector<Angles> &s, int j)
{
    double m = 0;
    for (size_t k = 1; k < s.size(); k++)
        m = std::max(m, std::abs(s[k].q[j] - s[k - 1].q[j]) / kDt);
    return m;
}

int ArgMax(const std::array<double, JOINTS_COUNT> &v)
{
    return int(std::max_element(v.begin(), v.end()) - v.begin());
}

class SpeedLimitTest : public SolverTest
{
protected:
    // applies the limit to the robot and rebuilds the solver from it
    void LimitSpeed(int joint, double rad_s)
    {
        robot->SetMaxJointSpeed(joint, rad_s);
        solver = NewSolver();
    }
    void LimitAcceleration(int joint, double rad_s2)
    {
        robot->SetMaxJointAcceleration(joint, rad_s2);
        solver = NewSolver();
    }
    ::testing::AssertionResult MoveJ(const Angles &from, const Angles &to, float v, float a, std::vector<Angles> &out)
    {
        std::vector<float> flat;
        auto r = RunsWithoutThrowing([&] { solver->MoveJ(from, to, v, a, kDtMs, flat); });
        out = Samples(flat);
        return r;
    }
    ::testing::AssertionResult MoveL(const StateVector &p1, const StateVector &p2, double v, double a, std::vector<Angles> &out)
    {
        std::vector<float> flat;
        auto r = RunsWithoutThrowing([&] { solver->MoveL(p1, p2, v, a, WorkPose(), kDtMs, flat); });
        out = Samples(flat);
        return r;
    }
    ::testing::AssertionResult MoveC(const StateVector &a, const StateVector &b, const StateVector &c, double v, double acc,
                                     std::vector<Angles> &out)
    {
        std::vector<float> flat;
        auto r = RunsWithoutThrowing([&] { solver->MoveC(WorkPose(), a, b, c, v, acc, kDtMs, flat); });
        out = Samples(flat);
        return r;
    }
    // the MoveLStraight scenario: 10 cm towards the base and 5 cm down
    void LineScenario(StateVector &p1, StateVector &p2)
    {
        p1 = ReferencePose(WorkPose());
        p2 = p1;
        p2.x -= 0.10;
        p2.z -= 0.05;
    }
    // the MoveCArc scenario: a half circle of radius 5 cm in the XZ plane
    void ArcScenario(StateVector &a, StateVector &b, StateVector &c)
    {
        a = ReferencePose(WorkPose());
        b = a;
        b.x -= 0.05;
        b.z -= 0.05;
        c = a;
        c.x -= 0.10;
    }
};

} // namespace

// ---------------------------------------------------------------------------
// MoveJ
// ---------------------------------------------------------------------------

TEST_F(SpeedLimitTest, MoveJWithinLimitsIsNotSlowedDown)
{
    Angles to = WorkPose();
    to.q[0] += 0.5;
    std::vector<Angles> s;
    ASSERT_TRUE(MoveJ(WorkPose(), to, 1.0f, 2.0f, s));
    const Solver::SpeedLimit info = solver->GetLastSpeedLimit();
    EXPECT_DOUBLE_EQ(info.scale, 1.0);
    EXPECT_EQ(info.joint, -1);
    EXPECT_NEAR(PeaksOf(s).speed[0], 1.0, 0.02);
}

TEST_F(SpeedLimitTest, MoveJRequestedSpeedIsCappedByTheJointLimit)
{
    LimitSpeed(0, 0.5);
    Angles to = WorkPose();
    to.q[0] += 1.0;
    std::vector<Angles> s;
    ASSERT_TRUE(MoveJ(WorkPose(), to, 1.0f, 2.0f, s));
    EXPECT_LE(PeaksOf(s).speed[0], 0.5 + 1e-6);
    // positions come out as float: differencing them costs ~1e-5 rad/s
    EXPECT_LE(MeasuredSpeed(s, 0), 0.5 + 1e-4);
    // it does use the speed it is allowed
    EXPECT_GT(PeaksOf(s).speed[0], 0.49);
    const Solver::SpeedLimit info = solver->GetLastSpeedLimit();
    EXPECT_NEAR(info.scale, 0.5, 1e-6);
    EXPECT_EQ(info.joint, 0);
    EXPECT_FALSE(info.acceleration);
    EXPECT_NEAR(s.back().q[0], to.q[0], 1e-3);
}

TEST_F(SpeedLimitTest, MoveJRequestedAccelerationIsCappedByTheJointLimit)
{
    LimitAcceleration(3, 1.0);
    Angles to = WorkPose();
    to.q[3] += 1.2;
    std::vector<Angles> s;
    ASSERT_TRUE(MoveJ(WorkPose(), to, 2.0f, 4.0f, s));
    EXPECT_LE(PeaksOf(s).accel[3], 1.0 + 1e-6);
    EXPECT_GT(PeaksOf(s).accel[3], 0.9);
    const Solver::SpeedLimit info = solver->GetLastSpeedLimit();
    EXPECT_EQ(info.joint, 3);
    EXPECT_TRUE(info.acceleration);
    EXPECT_LT(info.scale, 1.0);
}

// A slow wrist joint sets the pace for the whole synchronous move: every joint
// stays within its own limit and the path stays a straight line in joint space.
TEST_F(SpeedLimitTest, MoveJSlowJointSlowsTheOthersAndKeepsThemInSync)
{
    LimitSpeed(4, 0.1);
    const Angles from = WorkPose();
    Angles to = from;
    to.q[0] += 0.5;
    to.q[4] += 0.3;
    std::vector<Angles> s;
    ASSERT_TRUE(MoveJ(from, to, 1.0f, 2.0f, s));
    const Peaks p = PeaksOf(s);
    const auto v_max = robot->GetMaxJointSpeeds();
    for (int j = 0; j < JOINTS_COUNT; j++)
        EXPECT_LE(p.speed[j], v_max[j] + 1e-6) << "joint " << j;
    EXPECT_GT(p.speed[4], 0.099);
    // joint 0 travels 5/3 of joint 4, so it moves 5/3 as fast
    EXPECT_NEAR(p.speed[0], p.speed[4] * 0.5 / 0.3, 1e-6);
    for (const Angles &a : s)
        EXPECT_NEAR((a.q[0] - from.q[0]) / 0.5, (a.q[4] - from.q[4]) / 0.3, 1e-6);
    EXPECT_EQ(solver->GetLastSpeedLimit().joint, 4);
}

// ---------------------------------------------------------------------------
// MoveL / MoveC
// ---------------------------------------------------------------------------

TEST_F(SpeedLimitTest, MoveLWithinLimitsIsNotSlowedDown)
{
    StateVector p1, p2;
    LineScenario(p1, p2);
    std::vector<Angles> s;
    ASSERT_TRUE(MoveL(p1, p2, 0.1, 0.5, s));
    EXPECT_DOUBLE_EQ(solver->GetLastSpeedLimit().scale, 1.0);
    EXPECT_EQ(solver->GetLastSpeedLimit().joint, -1);
}

TEST_F(SpeedLimitTest, MoveLJointSpeedLimitSlowsTheMoveButKeepsTheLine)
{
    StateVector p1, p2;
    LineScenario(p1, p2);
    std::vector<Angles> free_run;
    ASSERT_TRUE(MoveL(p1, p2, 0.1, 0.5, free_run));
    const Peaks free_peaks = PeaksOf(free_run);
    const int j = ArgMax(free_peaks.speed);
    const double limit = 0.5 * free_peaks.speed[j];

    LimitSpeed(j, limit);
    std::vector<Angles> s;
    ASSERT_TRUE(MoveL(p1, p2, 0.1, 0.5, s));
    EXPECT_LE(PeaksOf(s).speed[j], limit * 1.01);
    EXPECT_LE(MeasuredSpeed(s, j), limit * 1.01);
    EXPECT_GT(s.size(), free_run.size() * 1.5);
    const Vector3 l1 = p1.GetCoord(), l2 = p2.GetCoord();
    for (const Angles &a : s)
        EXPECT_LT(CalcDistancePointToLine(ReferenceTcp(a), &l1, &l2), 1e-4);
    EXPECT_VEC3_NEAR(ReferenceTcp(s.back()), p2.GetCoord(), 1e-5);
    const Solver::SpeedLimit info = solver->GetLastSpeedLimit();
    EXPECT_EQ(info.joint, j);
    EXPECT_FALSE(info.acceleration);
    EXPECT_GT(info.scale, 0.4);
    EXPECT_LT(info.scale, 0.55);
}

TEST_F(SpeedLimitTest, MoveLJointAccelerationLimitSlowsTheMove)
{
    StateVector p1, p2;
    LineScenario(p1, p2);
    std::vector<Angles> free_run;
    // short accelerations dominate: a low TCP acceleration over a long line
    ASSERT_TRUE(MoveL(p1, p2, 0.1, 0.5, free_run));
    const Peaks free_peaks = PeaksOf(free_run);
    const int j = ArgMax(free_peaks.accel);
    const double limit = 0.25 * free_peaks.accel[j];

    LimitAcceleration(j, limit);
    std::vector<Angles> s;
    ASSERT_TRUE(MoveL(p1, p2, 0.1, 0.5, s));
    EXPECT_LE(PeaksOf(s).accel[j], limit * 1.02);
    EXPECT_GT(s.size(), free_run.size());
    EXPECT_VEC3_NEAR(ReferenceTcp(s.back()), p2.GetCoord(), 1e-5);
    const Solver::SpeedLimit info = solver->GetLastSpeedLimit();
    EXPECT_EQ(info.joint, j);
    EXPECT_TRUE(info.acceleration);
    EXPECT_LT(info.scale, 1.0);
}

TEST_F(SpeedLimitTest, MoveCJointSpeedLimitSlowsTheArc)
{
    StateVector a, b, c;
    ArcScenario(a, b, c);
    std::vector<Angles> free_run;
    ASSERT_TRUE(MoveC(a, b, c, 0.1, 0.5, free_run));
    const Peaks free_peaks = PeaksOf(free_run);
    const int j = ArgMax(free_peaks.speed);
    const double limit = 0.5 * free_peaks.speed[j];

    LimitSpeed(j, limit);
    std::vector<Angles> s;
    ASSERT_TRUE(MoveC(a, b, c, 0.1, 0.5, s));
    EXPECT_LE(PeaksOf(s).speed[j], limit * 1.01);
    EXPECT_GT(s.size(), free_run.size() * 1.5);
    EXPECT_VEC3_NEAR(ReferenceTcp(s.back()), c.GetCoord(), 1e-5);
    EXPECT_EQ(solver->GetLastSpeedLimit().joint, j);
    EXPECT_LT(solver->GetLastSpeedLimit().scale, 0.55);
}

// A limit that would need the move to crawl (e.g. right through a
// singularity) is reported instead of planning a near-endless move.
TEST_F(SpeedLimitTest, MoveLThatWouldHaveToCrawlThrows)
{
    for (int j = 0; j < JOINTS_COUNT; j++)
        robot->SetMaxJointSpeed(j, 1e-5);
    solver = NewSolver();
    StateVector p1, p2;
    LineScenario(p1, p2);
    std::vector<float> flat;
    try
    {
        ScopedSilence quiet;
        solver->MoveL(p1, p2, 0.1, 0.5, WorkPose(), kDtMs, flat);
        FAIL() << "MoveL did not throw";
    }
    catch (const std::runtime_error &e)
    {
        EXPECT_NE(std::string(e.what()).find("speed limit"), std::string::npos) << e.what();
    }
}
