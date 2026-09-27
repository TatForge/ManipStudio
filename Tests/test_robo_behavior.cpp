// Phase 2: behaviour of Robo -- forward kinematics against an independent
// reference, the Jacobian against numerical differentiation, the DLS inverse
// kinematics loop, settings persistence and the (legacy) dynamics.

#include "gtest/gtest.h"
#include "robo.h"
#include "test_helpers.h"

#include <QFile>
#include <QSettings>
#include <QTemporaryDir>

#include <cmath>
#include <memory>

using namespace RoboMath;
using namespace test_helpers;

// Robo declares `friend class Trajectory;` (the class itself was never
// written). Defining it here gives the tests read/write access to Robo's
// internal state without touching production code.
class Trajectory
{
public:
    static Matrix &State(Robo &r) { return *r.state_matrix; }
    static Vector &DeltaThetas(Robo &r) { return *r.delta_thetas; }
    static Matrix &Jacobian(Robo &r) { return *r.jacobian; }
    static Matrix4x4 &T0(Robo &r, int i) { return r.T0[i]; }
    static void UpdateTMatrix(Robo &r) { r.UpdateTMatrix(); }
    static scalar Mass(Robo &r, int i) { return r.m_mass[i]; }
    static Vector &CenterOfMass(Robo &r, int i) { return r.m_cm[i]; }
    static Tensor &Inertia(Robo &r, int i) { return r.m_i[i]; }

    // Puts the robot into joint configuration q with a clean (zero)
    // delta_thetas vector, working around the uninitialised-delta_thetas bug
    // (see RoboTest.LoadManipReadsRealDHThetasFromIniFile).
    static void SetJoints(Robo &r, const Angles &q)
    {
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            r.delta_thetas->at(i) = 0;
            (*r.state_matrix)(i, 3) = q.q[i];
        }
        r.UpdateTMatrix();
    }
};

namespace {

class RoboBehavior : public ::testing::Test
{
protected:
    void SetUp() override
    {
        chdir_guard = std::make_unique<ScopedChdir>(TEST_PROJECT_DIR);
        robot = std::make_unique<Robo>(6);
        ASSERT_TRUE(robot->LoadManip());
    }
    void TearDown() override
    {
        robot.reset();
        chdir_guard.reset();
    }

    Angles CurrentJoints()
    {
        Angles a;
        for (int i = 0; i < JOINTS_COUNT; i++)
            a.q[i] = robot->GetTheta(i);
        return a;
    }

    std::unique_ptr<ScopedChdir> chdir_guard;
    std::unique_ptr<Robo> robot;
};

const std::vector<Angles> &PoseSet()
{
    static const std::vector<Angles> poses{
        MakeAngles({0, 0, 0, 0, 0, 0}),
        MakeAngles({0.3, -0.5, 0.9, 0.2, -0.4, 0.1}),
        MakeAngles({-1.2, -M_PI / 4, M_PI / 2, -M_PI + M_PI / 3.7, -M_PI / 2, 0}), // Solver::GetExtarnalPosition
        MakeAngles({0, -M_PI / 4, M_PI / 2, M_PI / 4, M_PI / 2, 0}),               // Solver::GetInternalPosition
        MakeAngles({2.0, 0.4, -1.1, 1.5, 0.8, -2.2}),
        MakeAngles({-0.7, -1.0, 1.4, -0.6, 1.2, 0.9})};
    return poses;
}

} // namespace

// Hand-derived from the DH table (see test_helpers.h): with every joint at 0
// the arm stretches along +X, the wrist offsets add +Y and the last two links
// point down.
TEST_F(RoboBehavior, ForwardKinematicZeroPoseMatchesHandCalculation)
{
    Matrix dh = ProjectDHMatrix();
    StateVector tcp = robot->SolveForwardKinematic(Angles(), &dh);
    EXPECT_NEAR(tcp.x, 0.375 + 0.295, 1e-12);
    EXPECT_NEAR(tcp.y, 0.1 - 0.1 + 0.1205 + 0.1226, 1e-12);
    EXPECT_NEAR(tcp.z, 0.2325 - 0.171, 1e-12);
}

TEST_F(RoboBehavior, ForwardKinematicMatchesReferenceImplementation)
{
    Matrix dh = ProjectDHMatrix();
    for (const auto &q : PoseSet())
    {
        StateVector tcp = robot->SolveForwardKinematic(q, &dh);
        EXPECT_VEC3_NEAR(tcp.GetCoord(), ReferenceTcp(q), 1e-12);
    }
}

// The roll/pitch/yaw returned by SolveForwardKinematic must describe the same
// orientation as the reference rotation matrix of the last frame.
// The zero pose and GetInternalPosition() share the same TCP rotation: an
// exact half turn about (1,-1,0) (x -> -Y, y -> -X, z -> -Z). Depending on
// last-bit rounding of the trace this takes the defective trace <= 0 branch
// of Matrix4x4::ConvertToQuaternion (see
// Rotations.Matrix4x4ToQuaternionForHalfTurns) and the reported orientation
// is off by 180 deg. Solver's IK uses the same conversion.
TEST_F(RoboBehavior, ForwardKinematicOrientationMatchesReference)
{
    Matrix dh = ProjectDHMatrix();
    for (const auto &q : PoseSet())
    {
        StateVector tcp = robot->SolveForwardKinematic(q, &dh);
        Quaternion ori;
        ori.EulerToQuat(tcp.GetAngles());
        Mat4 ref = ReferenceT0(q.q, JOINTS_COUNT - 1);
        for (int col = 0; col < 3; col++)
        {
            Vector3 e(col == 0, col == 1, col == 2);
            EXPECT_VEC3_NEAR(ori.RotateVector(e), Vector3(ref[0][col], ref[1][col], ref[2][col]), 1e-9)
                << "axis " << col;
        }
    }
}

TEST_F(RoboBehavior, UpdateTMatrixMatchesReferenceFrames)
{
    for (const auto &q : PoseSet())
    {
        Trajectory::SetJoints(*robot, q);
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            Mat4 ref = ReferenceT0(q.q, j);
            const Matrix4x4 &t0 = Trajectory::T0(*robot, j);
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    EXPECT_NEAR(t0.at(r, c), ref[r][c], 1e-12) << "frame " << j << " (" << r << "," << c << ")";
        }
    }
}

// Linear rows: dp/dq_i by central differences of the reference FK.
// Angular rows: world z-axis of joint frame i.
TEST_F(RoboBehavior, JacobianMatchesNumericalDerivative)
{
    const double h = 1e-6;
    for (const auto &q : PoseSet())
    {
        Trajectory::SetJoints(*robot, q);
        robot->CalculateJacobian();
        const Matrix &J = Trajectory::Jacobian(*robot);
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            Angles qp = q, qm = q;
            qp.q[i] += h;
            qm.q[i] -= h;
            Vector3 dp = (ReferenceTcp(qp) - ReferenceTcp(qm)) / (2 * h);
            EXPECT_VEC3_NEAR(Vector3(J.at(0, i), J.at(1, i), J.at(2, i)), dp, 1e-7) << "column " << i;
            Mat4 frame = ReferenceT0(q.q, i);
            EXPECT_VEC3_NEAR(Vector3(J.at(3, i), J.at(4, i), J.at(5, i)), Vector3(frame[0][2], frame[1][2], frame[2][2]), 1e-12)
                << "column " << i;
        }
    }
}

TEST_F(RoboBehavior, SingleDampedLeastSquaresStepReducesPositionError)
{
    Angles q = PoseSet()[1];
    Trajectory::SetJoints(*robot, q);
    Vector3 target = ReferenceTcp(q) + Vector3(0.02, -0.01, 0.015);
    Matrix dh = ProjectDHMatrix();
    Vector3 target_r = robot->SolveForwardKinematic(q, &dh).GetAngles();

    robot->CalculateDeltaS(target);
    robot->CalculateDeltaR(target_r);
    const double error_before = robot->GetDeltaS().Length();
    EXPECT_NEAR(robot->GetDeltaR().Length(), 0, 1e-9);

    robot->CalculateJacobian();
    robot->CalculateDeltaThetas();
    Trajectory::UpdateTMatrix(*robot); // applies delta_thetas
    robot->CalculateDeltaS(target);
    EXPECT_LT(robot->GetDeltaS().Length(), 0.3 * error_before);
}

TEST_F(RoboBehavior, DeltaThetasAreClampedToMaxAngle)
{
    Angles q = PoseSet()[1];
    Trajectory::SetJoints(*robot, q);
    robot->CalculateDeltaS(ReferenceTcp(q) + Vector3(5, 5, 5)); // absurdly far target
    robot->CalculateDeltaR(Vector3(0, 0, 0));
    robot->CalculateJacobian();
    robot->CalculateDeltaThetas();
    EXPECT_LE(Trajectory::DeltaThetas(*robot).MaxAbs(), MaxAngleDls + 1e-12);
}

// Full IK loop: the robot must end up in a configuration whose *actual*
// forward kinematics is the requested pose.
TEST_F(RoboBehavior, SolveInverseKinematicReachesRequestedPose)
{
    Matrix dh = ProjectDHMatrix();
    Angles start = PoseSet()[1];
    Angles goal = start;
    goal.q[0] += 0.15;
    goal.q[1] -= 0.1;
    goal.q[2] += 0.1;
    goal.q[4] += 0.2;
    StateVector target = robot->SolveForwardKinematic(goal, &dh);

    Trajectory::SetJoints(*robot, start);
    {
        ScopedSilence quiet;
        robot->SolveInverseKinematic(1e-6, target.GetCoord(), target.GetAngles());
    }
    Angles reached = CurrentJoints();
    EXPECT_VEC3_NEAR(ReferenceTcp(reached), target.GetCoord(), 1e-5);
    StateVector reached_pose = robot->SolveForwardKinematic(reached, &dh);
    Quaternion q_reached, q_target;
    q_reached.EulerToQuat(reached_pose.GetAngles());
    q_target.EulerToQuat(target.GetAngles());
    for (const auto &v : {unit_x, unit_y, unit_z})
        EXPECT_VEC3_NEAR(q_reached.RotateVector(v), q_target.RotateVector(v), 1e-4);
}

// CalculateDeltaR must return the world-frame rotation vector (axis * angle)
// that turns the current TCP orientation into the target one. The IK loops
// of Robo and Solver drive this vector to zero.
TEST_F(RoboBehavior, CalculateDeltaRIsRotationVectorToTarget)
{
    Angles q = PoseSet()[1];
    Trajectory::SetJoints(*robot, q);
    Mat4 current = ReferenceT0(q.q, JOINTS_COUNT - 1);
    for (const auto &axis : {Vector3(0, 0, 1), Vector3(1, 0, 0), Vector3(0.3, -0.8, 0.5)})
        for (double angle : {0.05, -0.2, 0.6})
        {
            Vector3 n = axis.Normalize();
            double c = cos(angle), s = sin(angle), t = 1 - c;
            Mat4 r = Identity4();
            r[0][0] = t * n.x * n.x + c;       r[0][1] = t * n.x * n.y - s * n.z; r[0][2] = t * n.x * n.z + s * n.y;
            r[1][0] = t * n.x * n.y + s * n.z; r[1][1] = t * n.y * n.y + c;       r[1][2] = t * n.y * n.z - s * n.x;
            r[2][0] = t * n.x * n.z - s * n.y; r[2][1] = t * n.y * n.z + s * n.x; r[2][2] = t * n.z * n.z + c;
            Mat4 target = Mul(r, current);
            Vector3 rpy(atan2(target[2][1], target[2][2]), -asin(target[2][0]), atan2(target[1][0], target[0][0]));
            robot->CalculateDeltaR(rpy);
            EXPECT_VEC3_NEAR(robot->GetDeltaR(), n * angle, 1e-9) << "angle " << angle;
        }
}

TEST_F(RoboBehavior, CheckTrajectoryForLinearAcceptsPathOnLine)
{
    Angles q = PoseSet()[1];
    Trajectory::SetJoints(*robot, q);
    Polynom hold; // a0 = q, every other coefficient 0 -> robot stands still
    for (int i = 0; i < JOINTS_COUNT; i++)
        hold.joint[i].a[0] = q.q[i];
    Vector3 p = ReferenceTcp(q);
    ScopedSilence quiet;
    EXPECT_TRUE(robot->CheckTrajectoryForLinear(hold, p - Vector3(0.1, 0, 0), p + Vector3(0.1, 0, 0), 100, 10, 1e-6));
}

TEST_F(RoboBehavior, CheckTrajectoryForLinearRejectsPathOffLine)
{
    Angles q = PoseSet()[1];
    Trajectory::SetJoints(*robot, q);
    Polynom hold;
    for (int i = 0; i < JOINTS_COUNT; i++)
        hold.joint[i].a[0] = q.q[i];
    Vector3 p = ReferenceTcp(q) + Vector3(0, 0, 0.01); // line 1 cm above the TCP
    ScopedSilence quiet;
    EXPECT_FALSE(robot->CheckTrajectoryForLinear(hold, p - Vector3(0.1, 0, 0), p + Vector3(0.1, 0, 0), 100, 10, 1e-3));
}

TEST_F(RoboBehavior, CalcPolynomMatchesPolynomCalcPolynom)
{
    Angles a0 = PoseSet()[1], a1 = PoseSet()[4];
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        a0.q_p[i] = 0.1 * i;
        a1.q_pp[i] = -0.05 * i;
    }
    Polynom from_robo, from_polynom;
    robot->CalcPolynom(&a0, &a1, 1.5, from_robo);
    from_polynom.CalcPolynom(&a0, &a1, 1500);
    for (int i = 0; i < JOINTS_COUNT; i++)
        for (int k = 0; k < 6; k++)
            EXPECT_NEAR(from_robo.joint[i].a[k], from_polynom.joint[i].a[k], 1e-9) << "joint " << i << " a" << k;
}

// Robo::CalcPolynomLine builds a full quintic (unlike Polynom's cubic
// version), so both boundary states are matched.
TEST_F(RoboBehavior, CalcPolynomLineMatchesBothBoundaryStates)
{
    Angles a0 = PoseSet()[3], a1 = PoseSet()[5];
    Polynom pol;
    robot->CalcPolynomLine(&a0, &a1, 2.0, pol);
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_NEAR(pol.GetAngle(i, 0), a0.q[i], 1e-12);
        EXPECT_NEAR(pol.GetAngle(i, 2000), a1.q[i], 1e-9);
        EXPECT_NEAR(pol.GetSpeed(i, 2000), a1.q_p[i], 1e-9);
    }
}

// Regression test for a fixed bug: SolveForwardDynamic accumulates into m_moments.p /
// dv / hikm with `+=` but never resets them (and `buff` in the D-term loop is
// never reset between k iterations), so calling it twice for the same state
// doubles the result.
TEST_F(RoboBehavior, SolveForwardDynamicIsRepeatable)
{
    Trajectory::SetJoints(*robot, PoseSet()[1]);
    robot->SolveForwardDynamic();
    Robo::Moments first = *robot->GetMoments();
    robot->SolveForwardDynamic();
    Robo::Moments second = *robot->GetMoments();
    for (int i = 0; i < JOINTS_COUNT; i++)
        EXPECT_NEAR(second.tao[i], first.tao[i], 1e-9 + 1e-9 * std::abs(first.tao[i])) << "joint " << i;
}

// ---------------------------------------------------------------------------
// Settings persistence
// ---------------------------------------------------------------------------

// Writes a robot's settings with WriteSettingsToFile() and loads them back
// with LoadManip() into a fresh robot. Everything that was written must come
// back.
class RoboSettingsRoundTrip : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ASSERT_TRUE(dir.isValid());
        chdir_guard = std::make_unique<ScopedChdir>(dir.path().toStdString());
        std::initializer_list<Elem> list = {Elem(Vector3(0, 0, 0), unit_z, 0),      Elem(Vector3(0, 0, 0.23), unit_y, 0),
                                            Elem(Vector3(0.375, 0, 0.23), unit_y, 0), Elem(Vector3(0.67, 0, 0.23), unit_x, 0),
                                            Elem(Vector3(0.79, 0, 0.23), unit_y, 0),  Elem(Vector3(0.96, 0, 0.23), unit_x, 0)};
        {
            ScopedSilence quiet;
            source = std::make_unique<Robo>(list); // uses RoboSettingsInit() defaults
        }
        source->WriteSettingsToFile();
        loaded = std::make_unique<Robo>(6);
        ASSERT_TRUE(loaded->LoadManip());
    }
    void TearDown() override
    {
        loaded.reset();
        source.reset();
        chdir_guard.reset();
    }

    QTemporaryDir dir;
    std::unique_ptr<ScopedChdir> chdir_guard;
    std::unique_ptr<Robo> source;
    std::unique_ptr<Robo> loaded;
};

TEST_F(RoboSettingsRoundTrip, DenavitHartenbergTableSurvives)
{
    Matrix &a = Trajectory::State(*source);
    Matrix &b = Trajectory::State(*loaded);
    for (int i = 0; i < JOINTS_COUNT; i++)
        for (int c = 0; c < 3; c++) // a, alpha, d (theta is corrupted by the delta_thetas bug)
            EXPECT_NEAR(b(i, c), a(i, c), 1e-12) << "row " << i << " col " << c;
}

TEST_F(RoboSettingsRoundTrip, MaxMomentsSurvive)
{
    EXPECT_EQ(loaded->GetMaxJointMoments(), source->GetMaxJointMoments());
}

TEST_F(RoboSettingsRoundTrip, CentersOfMassAndInertiaSurvive)
{
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        for (int k = 0; k < 4; k++)
            EXPECT_NEAR(Trajectory::CenterOfMass(*loaded, i).at(k), Trajectory::CenterOfMass(*source, i).at(k), 1e-12);
        Tensor &a = Trajectory::Inertia(*source, i);
        Tensor &b = Trajectory::Inertia(*loaded, i);
        EXPECT_NEAR(b.xx(), a.xx(), 1e-15);
        EXPECT_NEAR(b.yy(), a.yy(), 1e-15);
        EXPECT_NEAR(b.zz(), a.zz(), 1e-15);
        EXPECT_NEAR(b.xy(), a.xy(), 1e-15);
        EXPECT_NEAR(b.xz(), a.xz(), 1e-15);
        EXPECT_NEAR(b.yz(), a.yz(), 1e-15);
    }
}

// Regression test for a fixed bug: WriteSettingsToFile() stores each link mass under
// the key "<i>" inside the "mass array", but LoadSettingsFromFile() reads the
// key "mass", which never exists -- every mass loads as 0 (which is also why
// ManipulatorSettigns.ini itself contains only zero masses).
TEST_F(RoboSettingsRoundTrip, JointLimitsAndFloorSurvive)
{
    // SetUp already wrote/loaded once; write custom values and load again.
    for (int i = 0; i < JOINTS_COUNT; i++)
        source->SetJointLimit(i, ToRad(-100.0 - i), ToRad(50.0 + 2 * i));
    source->SetFloorHeight(-0.12);
    source->WriteSettingsToFile();
    Robo reloaded(6);
    ASSERT_TRUE(reloaded.LoadManip());
    auto limits = reloaded.GetJointLimits();
    ASSERT_EQ(limits.size(), 6u);
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_NEAR(limits[i].first, ToRad(-100.0 - i), 1e-9) << "joint " << i;
        EXPECT_NEAR(limits[i].second, ToRad(50.0 + 2 * i), 1e-9) << "joint " << i;
    }
    EXPECT_NEAR(reloaded.GetFloorHeight(), -0.12, 1e-12);
}

TEST_F(RoboSettingsRoundTrip, MassesSurvive)
{
    for (int i = 0; i < JOINTS_COUNT; i++)
        EXPECT_NEAR(Trajectory::Mass(*loaded, i), Trajectory::Mass(*source, i), 1e-12) << "link " << i;
}

// Joint limits / floor height are optional in the settings file; without
// them the documented defaults apply (degrees):
// base +-170, shoulder -150..+30, elbow +-175, wrist +-180, +-135, flange +-360,
// floor at the mounting plane z = 0.
TEST(RoboSettings, DefaultJointLimitsAndFloor)
{
    Robo r(6);
    auto limits = r.GetJointLimits();
    ASSERT_EQ(limits.size(), 6u);
    const double expected[6][2] = {{-170, 170}, {-150, 30}, {-175, 175}, {-180, 180}, {-135, 135}, {-360, 360}};
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_NEAR(limits[i].first, ToRad(expected[i][0]), 1e-12) << "joint " << i;
        EXPECT_NEAR(limits[i].second, ToRad(expected[i][1]), 1e-12) << "joint " << i;
    }
    EXPECT_DOUBLE_EQ(r.GetFloorHeight(), 0.0);
}

TEST(RoboSettings, LoadsJointLimitsAndFloorFromIni)
{
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    ScopedChdir guard(dir.path().toStdString());
    ASSERT_TRUE(QFile::copy(QString(TEST_PROJECT_DIR) + "ManipulatorSettigns.ini", "ManipulatorSettigns.ini"));
    QFile::setPermissions("ManipulatorSettigns.ini", QFile::ReadOwner | QFile::WriteOwner);
    {
        QSettings s("ManipulatorSettigns.ini", QSettings::IniFormat);
        s.beginGroup("joint limits");
        s.setValue("1/min_deg", -90.0);
        s.setValue("1/max_deg", 10.0);
        s.endGroup();
        s.setValue("workspace/floor_z", 0.05);
        s.sync();
    }
    Robo r(6);
    ASSERT_TRUE(r.LoadManip());
    auto limits = r.GetJointLimits();
    EXPECT_NEAR(limits[1].first, ToRad(-90.0), 1e-12);
    EXPECT_NEAR(limits[1].second, ToRad(10.0), 1e-12);
    EXPECT_NEAR(limits[0].second, ToRad(170.0), 1e-12); // not given -> default
    EXPECT_NEAR(r.GetFloorHeight(), 0.05, 1e-12);
}

TEST(RoboSettings, ProjectIniDeclaresJointLimits)
{
    QSettings s(QString(TEST_PROJECT_DIR) + "ManipulatorSettigns.ini", QSettings::IniFormat);
    s.beginGroup("joint limits");
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_TRUE(s.contains(QString::number(i) + "/min_deg")) << "joint " << i;
        EXPECT_TRUE(s.contains(QString::number(i) + "/max_deg")) << "joint " << i;
    }
    s.endGroup();
    EXPECT_TRUE(s.contains("workspace/floor_z"));
}

TEST(RoboSettings, LoadManipThrowsWhenSectionMissing)
{
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    ScopedChdir guard(dir.path().toStdString());
    ASSERT_TRUE(QFile::copy(QString(TEST_PROJECT_DIR) + "ManipulatorSettigns.ini", "ManipulatorSettigns.ini"));
    QFile::setPermissions("ManipulatorSettigns.ini", QFile::ReadOwner | QFile::WriteOwner);
    {
        QSettings s("ManipulatorSettigns.ini", QSettings::IniFormat);
        s.remove("moments");
        s.sync();
    }
    Robo r(6);
    EXPECT_THROW(r.LoadManip(), std::runtime_error);
    // LoadSettingsFromFile() has assigned manip_settings before throwing, so r
    // is safe to destroy.
}

// Regression test for a fixed bug: when the settings file is missing,
// LoadManip() throws *before* manip_settings is assigned, and ~Robo() used to
// delete that uninitialised pointer afterwards.
TEST(RoboSettings, LoadManipThrowsWhenFileMissing)
{
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    ScopedChdir guard(dir.path().toStdString());
    Robo r(6);
    EXPECT_THROW(r.LoadManip(), std::runtime_error);
}
