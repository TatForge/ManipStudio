#ifndef SOLVER_FIXTURE_H
#define SOLVER_FIXTURE_H

// Common fixture and helpers for the Solver test files.

#include "solver_test_access.h"
#include "test_helpers.h"

#include <functional>
#include <memory>
#include <vector>

namespace solver_test {

using namespace RoboMath;
using namespace test_helpers;

constexpr int kAnglesFloats = 3 * JOINTS_COUNT; // q[6], q_p[6], q_pp[6] per sample

// A generic, collision-free working pose with the elbow bent, away from
// singularities and from the half-turn TCP orientation that breaks
// Matrix4x4::ConvertToQuaternion (see Rotations.Matrix4x4ToQuaternionForHalfTurns).
inline Angles WorkPose()
{
    return MakeAngles({0.3, -0.5, 0.9, 0.2, -0.4, 0.1});
}

class SolverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        chdir_guard = std::make_unique<ScopedChdir>(TEST_PROJECT_DIR);
        robot = std::make_unique<Robo>(6);
        ASSERT_TRUE(robot->LoadManip());
        solver = NewSolver();
    }
    void TearDown() override
    {
        {
            ScopedSilence quiet;
            solver.reset();
        }
        robot.reset();
        chdir_guard.reset();
    }

    std::unique_ptr<Solver> NewSolver()
    {
        std::unique_ptr<Solver> s;
        {
            ScopedSilence quiet; // SharedMem prints "can't attach" on every construction
            s = std::make_unique<Solver>(*robot);
        }
        // Workaround: Emulator::iter is never initialised (see
        // EmulatorTest.IterStartsAtZero); the Solver calls into the emulator
        // when it detects a collision, which then crashes on the garbage index.
        s->m_emulator->iter = 0;
        return s;
    }

    // TCP pose of joint configuration q as reported by the Solver's own FK
    // (position + roll/pitch/yaw via Matrix4x4::ConvertToQuaternion).
    StateVector SolverPose(const Angles &q)
    {
        Matrix4x4 TR[JOINTS_COUNT];
        solver->SolveDirectKinematic(q, TR);
        return StateVector(Position(TR[5]), TR[5].ConvertToQuaternion().GetEulerAngles());
    }

    std::unique_ptr<ScopedChdir> chdir_guard;
    std::unique_ptr<Robo> robot;
    std::unique_ptr<Solver> solver;
};

// Splits the flat float vector produced by MoveJ/MoveL/MoveC into samples.
inline std::vector<Angles> Samples(const std::vector<float> &flat)
{
    std::vector<Angles> out(flat.size() / kAnglesFloats);
    for (size_t k = 0; k < out.size(); k++)
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            out[k].q[j] = flat[k * kAnglesFloats + j];
            out[k].q_p[j] = flat[k * kAnglesFloats + JOINTS_COUNT + j];
            out[k].q_pp[j] = flat[k * kAnglesFloats + 2 * JOINTS_COUNT + j];
        }
    return out;
}

// World rotation of the TCP from the independent reference FK.
inline Mat4 ReferenceTcpFrame(const Angles &a)
{
    return ReferenceT0(a.q, JOINTS_COUNT - 1);
}

// ZYX roll/pitch/yaw of a rotation matrix (independent of Quaternion code).
inline Vector3 EulerOf(const Mat4 &m)
{
    return Vector3(atan2(m[2][1], m[2][2]), -asin(std::max(-1.0, std::min(1.0, m[2][0]))), atan2(m[1][0], m[0][0]));
}

// True TCP pose (position + ZYX Euler) from the reference FK.
inline StateVector ReferencePose(const Angles &a)
{
    Mat4 m = ReferenceTcpFrame(a);
    return StateVector(Position(m), EulerOf(m));
}

// Angle (rad) of the relative rotation between two reference frames.
inline double RotationDistance(const Mat4 &a, const Mat4 &b)
{
    double tr = 0;
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++)
            tr += a[k][i] * b[k][i];
    return acos(std::max(-1.0, std::min(1.0, (tr - 1) / 2)));
}

inline Mat4 EulerMatrix(const Vector3 &rpy)
{
    Mat4 ry = Identity4();
    ry[0][0] = cos(rpy.y);  ry[0][2] = sin(rpy.y);
    ry[2][0] = -sin(rpy.y); ry[2][2] = cos(rpy.y);
    return Mul(Mul(RotZ(rpy.z), ry), RotX(rpy.x));
}

// Runs fn and converts the exception types thrown by Solver (std::exception
// and, from CheckTrajectoryForLinear, QString) into a test failure message.
inline ::testing::AssertionResult RunsWithoutThrowing(const std::function<void()> &fn)
{
    try
    {
        ScopedSilence quiet;
        fn();
    }
    catch (const std::exception &e)
    {
        return ::testing::AssertionFailure() << "threw std::exception: " << e.what();
    }
    catch (const QString &e)
    {
        return ::testing::AssertionFailure() << "threw QString: " << e.toStdString();
    }
    catch (...)
    {
        return ::testing::AssertionFailure() << "threw unknown exception";
    }
    return ::testing::AssertionSuccess();
}

} // namespace solver_test

#endif // SOLVER_FIXTURE_H
