// Phase 4: PhysicsLib -- the public float-vector API of libManipPhysics.
//
// Conventions from physicslib.h: joint states are 18 floats
// (q[6], q_p[6], q_pp[6]); Cartesian points are 6 floats
// (x, y, z, roll, pitch, yaw). MoveL / MoveC / InverseKinematic take the
// angles of a point in DEGREES (they apply ToRad), and DirectKinematic
// returns the TCP angles in degrees too.

#include "gtest/gtest.h"
#include "physicslib.h"
#include "solver.h"
#include "test_helpers.h"

#include <QString>

#include <cstring>
#include <filesystem>
#include <memory>
#include <new>

using namespace RoboMath;
using namespace test_helpers;

// Free helpers defined (non-static) in physicslib.cpp.
Angles VectorToAngle(const fvector &v);
std::vector<float> AngleToVector(const Angles &a);

namespace {

constexpr int kAnglesFloats = 18;

fvector ToVec(const Angles &a)
{
    fvector v(kAnglesFloats, 0);
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        v[j] = a.q[j];
        v[j + 6] = a.q_p[j];
        v[j + 12] = a.q_pp[j];
    }
    return v;
}

Angles QOf(const fvector &flat, size_t sample)
{
    Angles a;
    for (int j = 0; j < JOINTS_COUNT; j++)
        a.q[j] = flat[sample * kAnglesFloats + j];
    return a;
}

// ZYX Euler of the reference TCP frame.
Vector3 ReferenceEuler(const Angles &q)
{
    Mat4 m = ReferenceT0(q.q, JOINTS_COUNT - 1);
    return Vector3(atan2(m[2][1], m[2][2]), -asin(m[2][0]), atan2(m[1][0], m[0][0]));
}

// Cartesian point in the API's format (angles in degrees).
fvector PointDeg(const Vector3 &p, const Vector3 &rpy)
{
    return {float(p.x), float(p.y), float(p.z), float(ToDeg(rpy.x)), float(ToDeg(rpy.y)), float(ToDeg(rpy.z))};
}

const Angles kWork = MakeAngles({0.3, -0.5, 0.9, 0.2, -0.4, 0.1});

class PhysicsLibTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        chdir_guard = std::make_unique<ScopedChdir>(TEST_PROJECT_DIR);
        lib = std::make_unique<PhysicsLib>();
        ScopedSilence quiet;
        lib->Initialize();
    }
    void TearDown() override
    {
        {
            ScopedSilence quiet;
            lib.reset();
        }
        chdir_guard.reset();
    }

    std::unique_ptr<ScopedChdir> chdir_guard;
    std::unique_ptr<PhysicsLib> lib;
};

} // namespace

TEST(PhysicsLibLifecycle, NotUsableBeforeInitialize)
{
    PhysicsLib lib;
    EXPECT_FALSE(lib.m_usable);
}

TEST_F(PhysicsLibTest, InitializeMakesLibraryUsable)
{
    EXPECT_TRUE(lib->m_usable);
}

TEST(PhysicsLibLifecycle, InitializeThrowsWithoutSettingsFile)
{
    EXPECT_EXIT(
        {
            std::filesystem::path empty = std::filesystem::temp_directory_path() / "manip_physicslib_empty";
            std::filesystem::create_directories(empty);
            std::filesystem::current_path(empty);
            PhysicsLib lib;
            try
            {
                lib.Initialize();
            }
            catch (const std::runtime_error &)
            {
                std::exit(0);
            }
            std::exit(1);
        },
        ::testing::ExitedWithCode(0), "");
}

// Regression test for a fixed bug: the constructor never initialises lib_data, and the
// destructor deletes it whenever it is not null -- destroying a PhysicsLib
// on which Initialize() was never called (or failed) deletes a garbage
// pointer. Constructed over a 0xAB-filled buffer to make it deterministic.
TEST(PhysicsLibLifecycleDeathTest, DestroyingUninitializedLibraryIsSafe)
{
    EXPECT_EXIT(
        {
            alignas(PhysicsLib) unsigned char storage[sizeof(PhysicsLib)];
            std::memset(storage, 0xAB, sizeof(storage));
            PhysicsLib *lib = new (storage) PhysicsLib;
            lib->~PhysicsLib();
            std::exit(0);
        },
        ::testing::ExitedWithCode(0), "");
}

TEST_F(PhysicsLibTest, DirectKinematicReturnsTcpPosition)
{
    for (const auto &q : {kWork, MakeAngles({1.1, -0.3, 0.8, 0.6, -0.9, -0.4})})
    {
        fvector T(6);
        lib->DirectKinematic(ToVec(q), T);
        EXPECT_VEC3_NEAR(Vector3(T[0], T[1], T[2]), ReferenceTcp(q), 1e-6);
    }
}

// Includes the zero pose and GetInternalPosition(), whose TCP rotation is a
// half turn. Through the float API neither currently reaches the defective
// branch of Matrix4x4::ConvertToQuaternion (pi/2 rounded to float is no
// longer an exact half turn), so this passes -- but only by rounding luck.
TEST_F(PhysicsLibTest, DirectKinematicReturnsTcpOrientation)
{
    for (const auto &q : {kWork, MakeAngles({1.1, -0.3, 0.8, 0.6, -0.9, -0.4}), MakeAngles({0, -M_PI / 4, M_PI / 2, M_PI / 4, M_PI / 2, 0}),
                          Angles()})
    {
        fvector T(6);
        lib->DirectKinematic(ToVec(q), T);
        Quaternion got, expected;
        got.EulerToQuat(Vector3(ToRad(T[3]), ToRad(T[4]), ToRad(T[5])));
        expected.EulerToQuat(ReferenceEuler(q));
        for (const auto &v : {unit_x, unit_y, unit_z})
            EXPECT_VEC3_NEAR(got.RotateVector(v), expected.RotateVector(v), 1e-5)
                << "q = " << q.q[0] << " " << q.q[1] << " " << q.q[2] << " " << q.q[3] << " " << q.q[4] << " " << q.q[5];
    }
}

// The joint angles themselves survive AngleToVector(): its `i + k * 3`
// offsets make later writes of q[3..5] overwrite the misplaced q_p values,
// so only the q_p / q_pp blocks are corrupted (see AngleVectorRoundTrip).
TEST_F(PhysicsLibTest, InverseKinematicReturnsJointAngles)
{
    Angles goal = MakeAngles({-0.4, -0.6, 1.2, -0.3, 0.7, 0.5});
    fvector target = PointDeg(ReferenceTcp(goal), ReferenceEuler(goal));
    fvector res;
    {
        ScopedSilence quiet;
        lib->InverseKinematic(target, ToVec(kWork), res);
    }
    ASSERT_EQ(res.size(), 18u);
    Angles got = QOf(res, 0);
    EXPECT_VEC3_NEAR(ReferenceTcp(got), ReferenceTcp(goal), 1e-4);
}

// The natural round trip through the public API: InverseKinematic(
// DirectKinematic(q)) must lead back to the same TCP pose, including the
// tool orientation (regression test: DirectKinematic used to report radians
// while InverseKinematic expects degrees).
TEST_F(PhysicsLibTest, InverseOfDirectKinematicRoundTrips)
{
    Angles q = MakeAngles({-0.4, -0.6, 1.2, -0.3, 0.7, 0.5});
    fvector pose(6), res;
    lib->DirectKinematic(ToVec(q), pose);
    Angles start = q;
    start.q[0] += 0.05;
    start.q[2] -= 0.05;
    try
    {
        ScopedSilence quiet;
        lib->InverseKinematic(pose, ToVec(start), res);
    }
    catch (const std::exception &e)
    {
        FAIL() << e.what();
    }
    ASSERT_EQ(res.size(), 18u);
    Angles got = QOf(res, 0);
    EXPECT_VEC3_NEAR(ReferenceTcp(got), ReferenceTcp(q), 1e-4);
    Quaternion q_got, q_expected;
    q_got.EulerToQuat(ReferenceEuler(got));
    q_expected.EulerToQuat(ReferenceEuler(q));
    for (const auto &v : {unit_x, unit_y, unit_z})
        EXPECT_VEC3_NEAR(q_got.RotateVector(v), q_expected.RotateVector(v), 1e-3) << "tool orientation not reproduced";
}

TEST(PhysicsLibConversions, AngleVectorRoundTrip)
{
    fvector v(18);
    for (int i = 0; i < 18; i++)
        v[i] = 0.1f * (i + 1);
    fvector back = AngleToVector(VectorToAngle(v));
    ASSERT_EQ(back.size(), 18u);
    for (int i = 0; i < 18; i++)
        EXPECT_FLOAT_EQ(back[i], v[i]) << "i = " << i;
}

// Regression test for a fixed bug: VectorToAngle() fills q_pp from v[i + 6] (the q_p
// block) instead of v[i + 12].
TEST(PhysicsLibConversions, VectorToAngleReadsAccelerationsFromLastBlock)
{
    fvector v(18, 0);
    for (int j = 0; j < 6; j++)
    {
        v[j] = 1;
        v[j + 6] = 2;
        v[j + 12] = 3;
    }
    Angles a = VectorToAngle(v);
    for (int j = 0; j < 6; j++)
    {
        EXPECT_FLOAT_EQ(a.q[j], 1);
        EXPECT_FLOAT_EQ(a.q_p[j], 2);
        EXPECT_FLOAT_EQ(a.q_pp[j], 3) << "joint " << j;
    }
}

TEST(PhysicsLibConversions, VectorToAngleRejectsShortInput)
{
    EXPECT_THROW(VectorToAngle(fvector(6, 0.f)), std::out_of_range);
}

TEST_F(PhysicsLibTest, MoveJProducesTrajectoryBetweenGivenAngles)
{
    Angles to = kWork;
    to.q[0] += 0.4;
    to.q[3] -= 0.3;
    fvector res;
    {
        ScopedSilence quiet;
        lib->MoveJ(ToVec(kWork), ToVec(to), 1.0f, 2.0f, res);
    }
    ASSERT_GT(res.size(), 0u);
    ASSERT_EQ(res.size() % kAnglesFloats, 0u);
    const size_t n = res.size() / kAnglesFloats;
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        EXPECT_NEAR(QOf(res, 0).q[j], kWork.q[j], 1e-6) << "joint " << j;
        EXPECT_NEAR(QOf(res, n - 1).q[j], to.q[j], 1e-3) << "joint " << j;
    }
}

TEST_F(PhysicsLibTest, MoveJRejectsShortAngleVectors)
{
    fvector res;
    EXPECT_THROW(lib->MoveJ(fvector(6, 0.f), fvector(6, 0.1f), 1.0f, 2.0f, res), std::out_of_range);
}

// Scenario through the API: straight 10 cm move towards the base, angles of
// the points given in degrees as documented.
TEST_F(PhysicsLibTest, MoveLFollowsStraightLine)
{
    Vector3 p = ReferenceTcp(kWork), e = ReferenceEuler(kWork);
    fvector p1 = PointDeg(p, e), p2 = PointDeg(p - Vector3(0.1, 0, 0), e);
    fvector res;
    try
    {
        ScopedSilence quiet;
        lib->MoveL(ToVec(kWork), p1, p2, 0.1f, 0.5f, res);
    }
    catch (const std::exception &ex)
    {
        FAIL() << ex.what();
    }
    catch (const QString &ex)
    {
        FAIL() << ex.toStdString();
    }
    ASSERT_GT(res.size(), 0u);
    ASSERT_EQ(res.size() % kAnglesFloats, 0u);
    // float points: the line is defined by the rounded coordinates
    Vector3 a(p1[0], p1[1], p1[2]), b(p2[0], p2[1], p2[2]);
    for (size_t k = 0; k < res.size() / kAnglesFloats; k++)
        EXPECT_LE(CalcDistancePointToLine(ReferenceTcp(QOf(res, k)), &a, &b), 1e-4) << "sample " << k;
    EXPECT_VEC3_NEAR(ReferenceTcp(QOf(res, res.size() / kAnglesFloats - 1)), b, 1e-3);
}

TEST_F(PhysicsLibTest, MoveCFollowsCircle)
{
    Vector3 p = ReferenceTcp(kWork), e = ReferenceEuler(kWork);
    fvector a = PointDeg(p, e), b = PointDeg(p + Vector3(-0.05, 0, -0.05), e), c = PointDeg(p + Vector3(-0.1, 0, 0), e);
    fvector res;
    try
    {
        ScopedSilence quiet;
        lib->MoveC(ToVec(kWork), a, b, c, 0.1f, 0.5f, res);
    }
    catch (const std::exception &ex)
    {
        FAIL() << ex.what();
    }
    catch (const QString &ex)
    {
        FAIL() << ex.toStdString();
    }
    ASSERT_GT(res.size(), 0u);
    Vector3 center(a[0] - 0.05f, a[1], a[2]);
    for (size_t k = 0; k < res.size() / kAnglesFloats; k++)
        EXPECT_NEAR(Dist(ReferenceTcp(QOf(res, k)), center), 0.05, 2e-4) << "sample " << k;
}

// The library must be a thin wrapper: for the same inputs it has to produce
// exactly what the Solver produces (guards the float/degree conversions).
class PhysicsLibVsSolver : public PhysicsLibTest
{
protected:
    void SetUp() override
    {
        PhysicsLibTest::SetUp();
        robot = std::make_unique<Robo>(6);
        ASSERT_TRUE(robot->LoadManip());
        ScopedSilence quiet;
        solver = std::make_unique<Solver>(*robot);
    }
    void TearDown() override
    {
        {
            ScopedSilence quiet;
            solver.reset();
        }
        robot.reset();
        PhysicsLibTest::TearDown();
    }
    static StateVector FromApi(const fvector &p)
    {
        return StateVector(p[0], p[1], p[2], ToRad(p[3]), ToRad(p[4]), ToRad(p[5]));
    }
    static Angles FromApiAngles(const fvector &v)
    {
        Angles a;
        for (int j = 0; j < 6; j++)
        {
            a.q[j] = v[j];
            a.q_p[j] = v[j + 6];
            a.q_pp[j] = v[j + 12];
        }
        return a;
    }
    std::unique_ptr<Robo> robot;
    std::unique_ptr<Solver> solver;
};

TEST_F(PhysicsLibVsSolver, MoveJIsIdentical)
{
    Angles to = kWork;
    to.q[1] -= 0.3;
    to.q[4] += 0.5;
    fvector from_lib, from_solver;
    ScopedSilence quiet;
    lib->MoveJ(ToVec(kWork), ToVec(to), 1.0f, 2.0f, from_lib);
    solver->MoveJ(FromApiAngles(ToVec(kWork)), FromApiAngles(ToVec(to)), 1.0f, 2.0f, 10, from_solver);
    ASSERT_EQ(from_lib.size(), from_solver.size());
    for (size_t i = 0; i < from_lib.size(); i++)
        ASSERT_FLOAT_EQ(from_lib[i], from_solver[i]) << "index " << i;
}

TEST_F(PhysicsLibVsSolver, MoveLIsIdentical)
{
    Vector3 p = ReferenceTcp(kWork), e = ReferenceEuler(kWork);
    fvector p1 = PointDeg(p, e), p2 = PointDeg(p - Vector3(0.05, 0, 0.03), e);
    fvector from_lib, from_solver;
    ScopedSilence quiet;
    lib->MoveL(ToVec(kWork), p1, p2, 0.1f, 0.5f, from_lib);
    solver->MoveL(FromApi(p1), FromApi(p2), 0.1f, 0.5f, FromApiAngles(ToVec(kWork)), 10, from_solver);
    ASSERT_EQ(from_lib.size(), from_solver.size());
    for (size_t i = 0; i < from_lib.size(); i++)
        ASSERT_FLOAT_EQ(from_lib[i], from_solver[i]) << "index " << i;
}

TEST_F(PhysicsLibVsSolver, MoveCIsIdentical)
{
    Vector3 p = ReferenceTcp(kWork), e = ReferenceEuler(kWork);
    fvector a = PointDeg(p, e), b = PointDeg(p + Vector3(-0.03, 0, -0.03), e), c = PointDeg(p + Vector3(-0.06, 0, 0), e);
    fvector from_lib, from_solver;
    ScopedSilence quiet;
    lib->MoveC(ToVec(kWork), a, b, c, 0.1f, 0.5f, from_lib);
    solver->MoveC(FromApiAngles(ToVec(kWork)), FromApi(a), FromApi(b), FromApi(c), 0.1f, 0.5f, 10, from_solver);
    ASSERT_EQ(from_lib.size(), from_solver.size());
    for (size_t i = 0; i < from_lib.size(); i++)
        ASSERT_FLOAT_EQ(from_lib[i], from_solver[i]) << "index " << i;
}

TEST_F(PhysicsLibVsSolver, DirectKinematicIsIdentical)
{
    fvector T(6);
    lib->DirectKinematic(ToVec(kWork), T);
    Matrix4x4 TR[6];
    solver->SolveDirectKinematic(FromApiAngles(ToVec(kWork)), TR);
    Vector3 rpy = TR[5].ConvertToQuaternion().GetEulerAngles();
    const double expected[6] = {TR[5].at(0, 3), TR[5].at(1, 3), TR[5].at(2, 3), ToDeg(rpy.x), ToDeg(rpy.y), ToDeg(rpy.z)};
    for (int i = 0; i < 6; i++)
        EXPECT_FLOAT_EQ(T[i], float(expected[i])) << "i = " << i;
}

TEST_F(PhysicsLibTest, PredefinedPositionsMatchSolver)
{
    fvector in = lib->GetInternalPosition(), ex = lib->GetExternalPosition();
    ASSERT_EQ(in.size(), 6u);
    ASSERT_EQ(ex.size(), 6u);
    const float in_expected[6] = {0, float(-M_PI / 4), float(M_PI / 2), float(M_PI / 4), float(M_PI / 2), 0};
    const float ex_expected[6] = {0, float(-M_PI / 4), float(M_PI / 2), float(-M_PI + M_PI / 3.7), float(-M_PI / 2), 0};
    for (int j = 0; j < 6; j++)
    {
        EXPECT_FLOAT_EQ(in[j], in_expected[j]);
        EXPECT_FLOAT_EQ(ex[j], ex_expected[j]);
    }
}
