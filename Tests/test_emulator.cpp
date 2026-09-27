// Phase 3: Emulator -- replays joint trajectories, computes the arm pose for
// the visualiser and publishes it through shared memory.

#include "solver_test_access.h"
#include "test_helpers.h"

#include <cstring>
#include <memory>
#include <new>

using namespace RoboMath;
using namespace test_helpers;

namespace {

class EmulatorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        chdir_guard = std::make_unique<ScopedChdir>(TEST_PROJECT_DIR);
        robot = std::make_unique<Robo>(6);
        ASSERT_TRUE(robot->LoadManip());
        {
            ScopedSilence quiet;
            emu = std::make_unique<Emulator>(*robot);
        }
    }
    void TearDown() override
    {
        {
            ScopedSilence quiet;
            emu.reset();
        }
        robot.reset();
        chdir_guard.reset();
    }

    static Polynom RestToRest(const Angles &a, const Angles &b, double t_ms)
    {
        Polynom p;
        p.CalcPolynom(&a, &b, t_ms);
        return p;
    }

    std::unique_ptr<ScopedChdir> chdir_guard;
    std::unique_ptr<Robo> robot;
    std::unique_ptr<Emulator> emu;
};

const Angles kA = MakeAngles({0.3, -0.5, 0.9, 0.2, -0.4, 0.1});
const Angles kB = MakeAngles({-0.4, -0.7, 1.3, -0.2, 0.5, 0.6});

} // namespace

// Regression test for a fixed bug: the public member `iter` (write index into the TCP
// trace buffer) is never initialised by the constructor. The first
// FillManipPositionData() then indexes points_to_visual with garbage.
// Constructed over a 0xAB-filled buffer to make the check deterministic.
TEST_F(EmulatorTest, IterStartsAtZero)
{
    alignas(Emulator) unsigned char storage[sizeof(Emulator)];
    std::memset(storage, 0xAB, sizeof(storage));
    Emulator *e;
    {
        ScopedSilence quiet;
        e = new (storage) Emulator(*robot);
    }
    int iter = e->iter;
    {
        ScopedSilence quiet;
        e->~Emulator();
    }
    EXPECT_EQ(iter, 0);
}

TEST_F(EmulatorTest, StartsFromRobotState)
{
    const Angles &ra = *robot->GetAngles();
    const auto &dh = ProjectDH();
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_DOUBLE_EQ(emu->m_emu_angles.q[j], ra.q[j]);
        EXPECT_DOUBLE_EQ(emu->state_matrix->at(j, 0), dh[j].a);
        EXPECT_DOUBLE_EQ(emu->state_matrix->at(j, 1), dh[j].alpha);
        EXPECT_DOUBLE_EQ(emu->state_matrix->at(j, 2), dh[j].d);
        EXPECT_DOUBLE_EQ(emu->state_matrix->at(j, 3), robot->GetTheta(j));
    }
}

TEST_F(EmulatorTest, MoveManipPolynomEndsAtTargetAngles)
{
    emu->MoveManipPolynom(RestToRest(kA, kB, 500), 500);
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_NEAR(emu->m_emu_angles.q[j], kB.q[j], 1e-9) << "joint " << j;
        EXPECT_NEAR(emu->m_emu_angles.q_p[j], 0, 1e-6) << "joint " << j;
    }
}

TEST_F(EmulatorTest, PublishedJointPositionsMatchForwardKinematics)
{
    emu->MoveManipPolynom(RestToRest(kA, kB, 300), 300);
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        Vector3 expected = Position(ReferenceT0(kB.q, j));
        Vector3 got(emu->manip_to_visual[3 * j], emu->manip_to_visual[3 * j + 1], emu->manip_to_visual[3 * j + 2]);
        EXPECT_VEC3_NEAR(got, expected, 1e-6) << "joint " << j;
    }
}

TEST_F(EmulatorTest, TcpTraceRecordsPointsEveryRangeToPlot)
{
    emu->MoveManipPolynom(RestToRest(kA, kB, 500), 500);
    const int points = emu->iter / 3;
    const double path = Dist(ReferenceTcp(kA), ReferenceTcp(kB));
    ASSERT_GE(points, 2);
    EXPECT_GE(points, static_cast<int>(path / RANGE_TO_PLOT)); // at least one point per 25 mm
    for (int k = 1; k < points; k++)
    {
        Vector3 p0(emu->points_to_visual[3 * k - 3], emu->points_to_visual[3 * k - 2], emu->points_to_visual[3 * k - 1]);
        Vector3 p1(emu->points_to_visual[3 * k], emu->points_to_visual[3 * k + 1], emu->points_to_visual[3 * k + 2]);
        EXPECT_GT(Dist(p0, p1), RANGE_TO_PLOT - 1e-6) << "point " << k;
    }
}

// Regression test for a fixed bug: FillManipPositionData() appends to the fixed
// 999-float points_to_visual buffer without any bound check, so a long
// enough session (here the arm swings back and forth 30 times) writes past
// the end of the buffer into the neighbouring members.
TEST_F(EmulatorTest, TcpTraceNeverOverflowsItsBuffer)
{
    for (int k = 0; k < 30; k++)
        emu->MoveManipPolynom(k % 2 ? RestToRest(kB, kA, 200) : RestToRest(kA, kB, 200), 200);
    EXPECT_LE(emu->iter, static_cast<int>(sizeof(emu->points_to_visual) / sizeof(float)));
}

TEST_F(EmulatorTest, SendDataToSharedPublishesManipulatorPose)
{
    emu->MoveManipPolynom(RestToRest(kA, kB, 100), 100);
    std::unique_ptr<SharedMem> reader;
    {
        ScopedSilence quiet;
        reader = std::make_unique<SharedMem>("manipulator", 18);
    }
    emu->SendDataToShared(MANIP, emu->manip_to_visual, sizeof(float) * 18);
    const float *got = static_cast<const float *>(reader->GetData());
    for (int i = 0; i < 18; i++)
        EXPECT_FLOAT_EQ(got[i], emu->manip_to_visual[i]) << "i = " << i;
}

// Regression test for a fixed bug: OneIterMoveLinear interpolates with fractions
// 0, 1/n, ..., (n-1)/n and never reaches the end angle of a segment, so after
// replaying a whole trajectory the emulated arm stops one sub-step short of
// the trajectory's final sample.
TEST_F(EmulatorTest, MoveManipLinearEndsAtFinalSample)
{
    std::vector<Angles> path{kA, kA, kA};
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        path[1].q[j] = 0.5 * (kA.q[j] + kB.q[j]);
        path[2].q[j] = kB.q[j];
    }
    std::vector<Vector3> speed(3);
    emu->MoveManipLinear(path, speed, 10);
    for (int j = 0; j < JOINTS_COUNT; j++)
        EXPECT_NEAR(emu->m_emu_angles.q[j], kB.q[j], 1e-9) << "joint " << j;
}

TEST_F(EmulatorTest, MoveMeshesPublishesEveryMeshVertex)
{
    emu->MoveManipPolynom(RestToRest(kA, kB, 100), 100);
    emu->MoveMeshes();
    size_t idx = 0;
    for (size_t m = 0; m < emu->meshes_buff.size(); m++)
        for (ushort i = 0; i < emu->meshes_buff[m].Size(); i++, idx += 3)
        {
            ASSERT_LT(idx + 2, sizeof(emu->meshes_to_visual) / sizeof(float));
            Vector3 p = emu->meshes_buff[m].point(i);
            EXPECT_NEAR(emu->meshes_to_visual[idx], p.x, 1e-6);
            EXPECT_NEAR(emu->meshes_to_visual[idx + 1], p.y, 1e-6);
            EXPECT_NEAR(emu->meshes_to_visual[idx + 2], p.z, 1e-6);
        }
    EXPECT_GT(idx, 0u);
}
