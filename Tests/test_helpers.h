#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

// Shared utilities for all test targets: chdir guard, stdout silencing,
// tolerance macros and an *independent* reference implementation of the
// manipulator's forward kinematics (modified / Craig DH, the convention used
// by Robo, Solver and Emulator) built only from std::array, so it can be used
// to verify the project's own Matrix4x4-based implementation.

#include "gtest/gtest.h"
#include "my_math.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <string>

namespace test_helpers {

// Robo::LoadSettingsFromFile() and the Solver/Emulator constructors use
// hardcoded relative paths ("ManipulatorSettigns.ini", "STLmodels/..."), so
// tests that construct them temporarily run from the project directory.
class ScopedChdir
{
public:
    explicit ScopedChdir(const std::filesystem::path &target)
        : m_previous(std::filesystem::current_path())
    {
        std::filesystem::current_path(target);
    }
    ~ScopedChdir()
    {
        std::filesystem::current_path(m_previous);
    }

private:
    std::filesystem::path m_previous;
};

// Much of the production code prints progress to stdout; swallow it so test
// output stays readable.
class ScopedSilence
{
public:
    ScopedSilence() { testing::internal::CaptureStdout(); }
    ~ScopedSilence() { testing::internal::GetCapturedStdout(); }
};

// DH table from ManipulatorSettigns.ini: columns a, alpha, d, theta.
struct DHRow
{
    double a;
    double alpha;
    double d;
};

inline const std::array<DHRow, JOINTS_COUNT> &ProjectDH()
{
    static const std::array<DHRow, JOINTS_COUNT> dh{{{0, -M_PI / 2, 0.2325},
                                                     {0.375, 0, 0.100},
                                                     {0.295, 0, -0.100},
                                                     {0, -M_PI / 2, 0.1205},
                                                     {0, M_PI / 2, 0.171},
                                                     {0, 0, 0.1226}}};
    return dh;
}

inline RoboMath::Matrix ProjectDHMatrix()
{
    const auto &dh = ProjectDH();
    RoboMath::Matrix m(JOINTS_COUNT, 4);
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        m(i, 0) = dh[i].a;
        m(i, 1) = dh[i].alpha;
        m(i, 2) = dh[i].d;
        m(i, 3) = 0;
    }
    return m;
}

using Mat4 = std::array<std::array<double, 4>, 4>;

inline Mat4 Identity4()
{
    Mat4 m{};
    for (int i = 0; i < 4; i++)
        m[i][i] = 1;
    return m;
}

inline Mat4 Mul(const Mat4 &a, const Mat4 &b)
{
    Mat4 r{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

inline Mat4 RotX(double t)
{
    Mat4 m = Identity4();
    m[1][1] = cos(t); m[1][2] = -sin(t);
    m[2][1] = sin(t); m[2][2] = cos(t);
    return m;
}

inline Mat4 RotZ(double t)
{
    Mat4 m = Identity4();
    m[0][0] = cos(t); m[0][1] = -sin(t);
    m[1][0] = sin(t); m[1][1] = cos(t);
    return m;
}

inline Mat4 Trans(double x, double y, double z)
{
    Mat4 m = Identity4();
    m[0][3] = x; m[1][3] = y; m[2][3] = z;
    return m;
}

// Link transform used by the project: T_i = RotX(alpha_{i-1}) * TransX(a_{i-1})
// * RotZ(q_i) * TransZ(d_i), with alpha_{-1} = a_{-1} = 0.
inline Mat4 LinkTransform(int i, double q)
{
    const auto &dh = ProjectDH();
    double alpha_prev = i == 0 ? 0 : dh[i - 1].alpha;
    double a_prev = i == 0 ? 0 : dh[i - 1].a;
    return Mul(Mul(RotX(alpha_prev), Trans(a_prev, 0, 0)), Mul(RotZ(q), Trans(0, 0, dh[i].d)));
}

// Base -> frame j transform (j = 0..5) for joint vector q.
inline Mat4 ReferenceT0(const double q[JOINTS_COUNT], int j)
{
    Mat4 t = Identity4();
    for (int i = 0; i <= j; i++)
        t = Mul(t, LinkTransform(i, q[i]));
    return t;
}

inline RoboMath::Vector3 Position(const Mat4 &m)
{
    return RoboMath::Vector3(m[0][3], m[1][3], m[2][3]);
}

inline RoboMath::Vector3 Position(const RoboMath::Matrix4x4 &m)
{
    return RoboMath::Vector3(m.at(0, 3), m.at(1, 3), m.at(2, 3));
}

inline RoboMath::Vector3 ReferenceTcp(const RoboMath::Angles &a)
{
    return Position(ReferenceT0(a.q, JOINTS_COUNT - 1));
}

inline RoboMath::Angles MakeAngles(std::initializer_list<double> q)
{
    RoboMath::Angles a;
    int i = 0;
    for (double v : q)
        a.q[i++] = v;
    return a;
}

inline double Dist(const RoboMath::Vector3 &a, const RoboMath::Vector3 &b)
{
    return (a - b).Length();
}

// Smallest signed difference between two angles, in (-pi, pi].
inline double AngleDiff(double a, double b)
{
    double d = std::fmod(a - b, 2 * M_PI);
    if (d > M_PI)
        d -= 2 * M_PI;
    if (d <= -M_PI)
        d += 2 * M_PI;
    return d;
}

inline testing::AssertionResult Vec3NearFormat(const char *actual_expr, const char *expected_expr, const char *,
                                               const RoboMath::Vector3 &actual, const RoboMath::Vector3 &expected, double tol)
{
    const double d = std::max({std::abs(actual.x - expected.x), std::abs(actual.y - expected.y), std::abs(actual.z - expected.z)});
    if (d <= tol)
        return testing::AssertionSuccess();
    return testing::AssertionFailure() << actual_expr << " = (" << actual.x << ", " << actual.y << ", " << actual.z << ")\n"
                                       << expected_expr << " = (" << expected.x << ", " << expected.y << ", " << expected.z << ")\n"
                                       << "max component difference " << d << " exceeds " << tol;
}

} // namespace test_helpers

// Streamable (supports `<< "message"`), like the built-in EXPECT_* macros.
#define EXPECT_VEC3_NEAR(actual, expected, tol) \
    EXPECT_PRED_FORMAT3(test_helpers::Vec3NearFormat, actual, expected, tol)
#define ASSERT_VEC3_NEAR(actual, expected, tol) \
    ASSERT_PRED_FORMAT3(test_helpers::Vec3NearFormat, actual, expected, tol)

#endif // TEST_HELPERS_H
