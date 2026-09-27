#include "gtest/gtest.h"
#include "robo.h"
#include "test_helpers.h"

#include <memory>
#include <cmath>

using namespace RoboMath;
using test_helpers::ScopedChdir;

class RoboTest : public ::testing::Test
{
protected:
    void SetUp() override { chdir_guard = std::make_unique<ScopedChdir>(TEST_PROJECT_DIR); }
    void TearDown() override { chdir_guard.reset(); }

    std::unique_ptr<ScopedChdir> chdir_guard;
};

TEST(RoboMoments, MakeNullZeroesAllFields)
{
    Robo::Moments m;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        m.tao[i] = 1;
        m.p[i] = 2;
        m.hikm[i] = 3;
        m.dv[i] = 4;
    }
    m.MakeNull();
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_DOUBLE_EQ(m.tao[i], 0);
        EXPECT_DOUBLE_EQ(m.p[i], 0);
        EXPECT_DOUBLE_EQ(m.hikm[i], 0);
        EXPECT_DOUBLE_EQ(m.dv[i], 0);
    }
}

// Regression test for a fixed bug: ~Robo() used to delete the never
// initialised manip_settings pointer when no settings were loaded or saved.
TEST(RoboLifecycle, RobotWithoutSettingsIsSafeToDestroy)
{
    EXPECT_EXIT(
        {
            {
                Robo r(6);
            }
            std::exit(0);
        },
        ::testing::ExitedWithCode(0), "");
}

TEST_F(RoboTest, LoadManipSucceeds)
{
    Robo r(6);
    EXPECT_TRUE(r.LoadManip());
}

// Regression test for a fixed high-impact bug: AllocateForNewRobot() allocates
// delta_thetas via `new Vector(n)`, which leaves its data uninitialized
// (garbage). LoadManip() correctly reads theta=0,0,0,0,0,1 from
// ManipulatorSettigns.ini into state_matrix, but then unconditionally calls
// UpdateTMatrix(), whose very first line is `A(i,3) += delta_thetas->at(i)`
// -- adding that uninitialized garbage straight into the just-loaded theta
// column. Every LoadManip() call (the normal startup path used throughout
// the app, e.g. main.cpp) corrupts the robot's joint angles with garbage.
// The exact garbage value differs run to run (uninitialized memory), but
// this test deterministically fails either way.
TEST_F(RoboTest, LoadManipReadsRealDHThetasFromIniFile)
{
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());
    for (int i = 0; i < JOINTS_COUNT - 1; i++)
        EXPECT_NEAR(r.GetTheta(i), 0.0, 1e-9);
    EXPECT_NEAR(r.GetTheta(JOINTS_COUNT - 1), 1.0, 1e-9);
}

TEST_F(RoboTest, GetMaxJointMomentsMatchesIniFile)
{
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());
    std::vector<scalar> moments = r.GetMaxJointMoments();
    ASSERT_EQ(moments.size(), 6u);
    EXPECT_DOUBLE_EQ(moments[0], 180);
    EXPECT_DOUBLE_EQ(moments[1], 180);
    EXPECT_DOUBLE_EQ(moments[2], 120);
    EXPECT_DOUBLE_EQ(moments[3], 120);
    EXPECT_DOUBLE_EQ(moments[4], 60);
    EXPECT_DOUBLE_EQ(moments[5], 60);
}

TEST_F(RoboTest, GetElemReturnsAllSixJoints)
{
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());
    for (int i = 0; i < JOINTS_COUNT; i++)
        ASSERT_NE(r.GetElem(i), nullptr);
}

TEST_F(RoboTest, GetEffectorIsAccessible)
{
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());
    ASSERT_NE(r.GetEffector(), nullptr);
}

// SolveForwardKinematic doesn't touch Robo's own state -- it's a pure
// function of its (Angles, state_matrix) arguments. Note the DH convention
// here is staggered: T[i]'s rotation part uses row (i-1)'s alpha (T[0]
// itself hardcodes a literal 0, ignoring row 0's own alpha entirely), so
// zeroing every row's alpha is what's needed to make T[1..5] genuinely
// identity, leaving only row 0's `d` offset -- a pure +Z translation that's
// trivial to verify by hand.
TEST_F(RoboTest, SolveForwardKinematicSingleLinkMatchesHandComputedTransform)
{
    // Robo(6) + LoadManip() here purely to get a safely-destructible object
    // (see the manip_settings note above) -- LoadSettingsFromFile() reads a
    // fixed size=6 array from the ini file regardless of number_of_elements,
    // so e.g. Robo(1) would make LoadManip() itself write out of bounds into
    // a 1-row state_matrix. This test's actual kinematics check below uses
    // its own independent DH table/angles, not r's loaded state.
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());

    Matrix dh{{0, 0, 0.2325, 0},
              {0, 0, 0, 0},
              {0, 0, 0, 0},
              {0, 0, 0, 0},
              {0, 0, 0, 0},
              {0, 0, 0, 0}};
    Angles a; // all zero
    StateVector result = r.SolveForwardKinematic(a, &dh);
    EXPECT_NEAR(result.x, 0.0, 1e-9);
    EXPECT_NEAR(result.y, 0.0, 1e-9);
    EXPECT_NEAR(result.z, 0.2325, 1e-9);
}

// Regression test for a fixed bug: under the CALC_INVERSE_MATRIX build path (defined
// unconditionally at the top of robo.cpp), ComputeJointsPosition()'s code
// that updates the `effector` member's position is compiled out (wrapped in
// #ifndef CALC_INVERSE_MATRIX), and neither LoadManip() nor UpdateTMatrix()
// ever write to `effector` directly -- only to the private T0[] matrices.
// So GetEffectorPosition() stays at its default-constructed (0,0,0)
// regardless of the robot's actual configuration, while the *real* forward
// kinematics result (independently computed here via the already-verified
// SolveForwardKinematic, using the same DH table LoadManip() reads) is a
// meaningfully different, non-zero position.
TEST_F(RoboTest, GetEffectorPositionReflectsActualForwardKinematics)
{
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());

    Matrix dh{{0, -M_PI / 2, 0.2325, 0},
              {0.375, 0, 0.100, 0},
              {0.295, 0, -0.100, 0},
              {0, -M_PI / 2, 0.1205, 0},
              {0, M_PI / 2, 0.171, 0},
              {0, 0, 0.1226, 0}};
    Angles a;
    a.q[5] = 1.0; // matches joint 6's theta=1 in ManipulatorSettigns.ini
    StateVector true_pose = r.SolveForwardKinematic(a, &dh);

    Vector3 reported = r.GetEffectorPosition();
    EXPECT_NEAR(reported.x, true_pose.x, 1e-6);
    EXPECT_NEAR(reported.y, true_pose.y, 1e-6);
    EXPECT_NEAR(reported.z, true_pose.z, 1e-6);
}

TEST_F(RoboTest, SolveInverseKinematicConvergesForNearbyTarget)
{
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());

    Matrix dh{{0, -M_PI / 2, 0.2325, 0},
              {0.375, 0, 0.100, 0},
              {0.295, 0, -0.100, 0},
              {0, -M_PI / 2, 0.1205, 0},
              {0, M_PI / 2, 0.171, 0},
              {0, 0, 0.1226, 0}};
    // Read back r's *actual* current joint angles via GetTheta() rather than
    // assuming they match ManipulatorSettigns.ini -- LoadManip() corrupts
    // them with uninitialized delta_thetas garbage (see the dedicated bug
    // test above), so this is the only way to know the real starting pose
    // and keep this test isolated to IK-convergence behaviour specifically.
    Angles a;
    for (int i = 0; i < JOINTS_COUNT; i++)
        a.q[i] = r.GetTheta(i);
    StateVector current = r.SolveForwardKinematic(a, &dh);

    Vector3 target(current.x + 0.001, current.y, current.z);
    Vector3 target_r(current.roll, current.pitch, current.yaw);
    const scalar accuracy = 1e-4;

    testing::internal::CaptureStdout();
    r.SolveInverseKinematic(accuracy, target, target_r);
    testing::internal::GetCapturedStdout();

    EXPECT_LE(r.GetDeltaR().Length() + r.GetDeltaS().Length(), accuracy * 2);
}

TEST_F(RoboTest, SolveForwardDynamicProducesFiniteMoments)
{
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());

    r.SolveForwardDynamic();
    Robo::Moments *moments = r.GetMoments();
    ASSERT_NE(moments, nullptr);
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_TRUE(std::isfinite(moments->tao[i])) << "joint " << i;
        EXPECT_TRUE(std::isfinite(moments->p[i])) << "joint " << i;
        EXPECT_TRUE(std::isfinite(moments->dv[i])) << "joint " << i;
        EXPECT_TRUE(std::isfinite(moments->hikm[i])) << "joint " << i;
    }
}

// ManipulatorSettigns.ini stores 0 for every link mass (the Solver keeps
// its own masses), so Robo's gravity moment term is exactly zero.
TEST_F(RoboTest, SolveForwardDynamicGravityMomentIsZeroForZeroIniMasses)
{
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());

    r.SolveForwardDynamic();
    Robo::Moments *moments = r.GetMoments();
    for (int i = 0; i < JOINTS_COUNT; i++)
        EXPECT_DOUBLE_EQ(moments->p[i], 0.0);
}
