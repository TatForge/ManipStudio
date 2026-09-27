#include "gtest/gtest.h"
#include "my_math.h"

#include <algorithm>
#include <cmath>

using namespace RoboMath;

namespace {

Angles MakeAngles(scalar q_base, scalar qp_base, scalar qpp_base)
{
    Angles a;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        a.q[i] = q_base + i * 0.1;
        a.q_p[i] = qp_base + i * 0.05;
        a.q_pp[i] = qpp_base + i * 0.02;
    }
    return a;
}

} // namespace

// A quintic (5th order) point-to-point trajectory is constructed so that
// position, velocity and acceleration all match the start state at t=0 and
// the end state at t=T -- this is the defining property of the standard
// quintic trajectory generator CalcPolynom implements.
TEST(Polynom, CalcPolynomMatchesBoundaryConditions)
{
    Angles a0 = MakeAngles(0.1, 0.2, 0.3);
    Angles a1 = MakeAngles(1.5, -0.4, -0.1);
    const scalar t_millisec = 1000;

    Polynom pol;
    pol.CalcPolynom(&a0, &a1, t_millisec);

    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_NEAR(pol.GetAngle(i, 0), a0.q[i], 1e-9);
        EXPECT_NEAR(pol.GetSpeed(i, 0), a0.q_p[i], 1e-9);
        EXPECT_NEAR(pol.GetAcceleration(i, 0), a0.q_pp[i], 1e-9);

        EXPECT_NEAR(pol.GetAngle(i, t_millisec), a1.q[i], 1e-6);
        EXPECT_NEAR(pol.GetSpeed(i, t_millisec), a1.q_p[i], 1e-6);
        EXPECT_NEAR(pol.GetAcceleration(i, t_millisec), a1.q_pp[i], 1e-6);
    }
}

// CalcPolynomLine is a cubic (only a[0..3] are set): it matches the full
// start state (position/velocity/acceleration) but only the end *position*
// -- end velocity/acceleration are not constrained by a cubic.
TEST(Polynom, CalcPolynomLineMatchesStartStateAndEndPosition)
{
    Angles a0 = MakeAngles(0.1, 0.2, 0.3);
    Angles a1 = MakeAngles(1.5, -0.4, -0.1);
    const int t_millisec = 1000;

    Polynom pol;
    pol.CalcPolynomLine(&a0, &a1, t_millisec);

    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_NEAR(pol.GetAngle(i, 0), a0.q[i], 1e-9);
        EXPECT_NEAR(pol.GetSpeed(i, 0), a0.q_p[i], 1e-9);
        EXPECT_NEAR(pol.GetAcceleration(i, 0), a0.q_pp[i], 1e-9);

        EXPECT_NEAR(pol.GetAngle(i, t_millisec), a1.q[i], 1e-6);
    }
}

TEST(Polynom, DefaultConstructedPolynomIsZeroEverywhere)
{
    Polynom pol;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_DOUBLE_EQ(pol.GetAngle(i, 500), 0);
        EXPECT_DOUBLE_EQ(pol.GetSpeed(i, 500), 0);
        EXPECT_DOUBLE_EQ(pol.GetAcceleration(i, 500), 0);
    }
}

TEST(Polynom, CoutDoesNotCrash)
{
    Polynom pol;
    testing::internal::CaptureStdout();
    pol.Cout("test");
    testing::internal::GetCapturedStdout();
}

// ---------------------------------------------------------------------------
// Behaviour of the quintic point-to-point profile for a rest-to-rest move --
// the textbook case with closed-form properties.
// ---------------------------------------------------------------------------

namespace {

Polynom RestToRest(scalar from, scalar to, scalar t_millisec)
{
    Angles a0, a1;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        a0.q[i] = from + 0.1 * i;
        a1.q[i] = to - 0.2 * i;
    }
    Polynom pol;
    pol.CalcPolynom(&a0, &a1, t_millisec);
    return pol;
}

} // namespace

TEST(Polynom, RestToRestStartsAndEndsWithZeroSpeedAndAcceleration)
{
    const scalar T = 2000;
    Polynom pol = RestToRest(-0.5, 1.2, T);
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_NEAR(pol.GetSpeed(i, 0), 0, 1e-12);
        EXPECT_NEAR(pol.GetSpeed(i, T), 0, 1e-9);
        EXPECT_NEAR(pol.GetAcceleration(i, 0), 0, 1e-12);
        EXPECT_NEAR(pol.GetAcceleration(i, T), 0, 1e-9);
    }
}

TEST(Polynom, RestToRestIsMonotonicAndSymmetric)
{
    const scalar T = 1500;
    Polynom pol = RestToRest(0.0, 1.0, T);
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        const scalar q0 = pol.GetAngle(i, 0), q1 = pol.GetAngle(i, T);
        const scalar dir = q1 > q0 ? 1 : -1;
        scalar prev = q0;
        for (scalar t = 10; t <= T; t += 10)
        {
            scalar q = pol.GetAngle(i, t);
            EXPECT_GE(dir * (q - prev), -1e-12) << "joint " << i << " t " << t;
            prev = q;
        }
        // Point symmetry about the midpoint.
        EXPECT_NEAR(pol.GetAngle(i, T / 2), (q0 + q1) / 2, 1e-9);
        EXPECT_NEAR(pol.GetAngle(i, 300) - q0, q1 - pol.GetAngle(i, T - 300), 1e-9);
    }
}

// Known closed form: peak speed of the quintic profile is 15/8 * dq / T,
// reached at T/2; peak acceleration is 10/sqrt(3) * dq / T^2.
TEST(Polynom, RestToRestPeakSpeedAndAccelerationMatchClosedForm)
{
    const scalar T = 1000;
    Polynom pol = RestToRest(0.2, 1.4, T);
    const scalar Ts = T / 1000.0;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        const scalar dq = pol.GetAngle(i, T) - pol.GetAngle(i, 0);
        if (std::abs(dq) < 1e-6)
            continue; // this joint doesn't move in RestToRest(); nothing to check
        EXPECT_NEAR(pol.GetSpeed(i, T / 2), 15.0 / 8.0 * dq / Ts, 1e-9);
        scalar max_acc = 0;
        for (scalar t = 0; t <= T; t += 1)
            max_acc = std::max(max_acc, std::abs(pol.GetAcceleration(i, t)));
        EXPECT_NEAR(max_acc, 10.0 / std::sqrt(3.0) * std::abs(dq) / (Ts * Ts), 1e-3 * std::abs(dq) / (Ts * Ts));
    }
}

TEST(Polynom, SpeedIsDerivativeOfAngle)
{
    Angles a0, a1;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        a0.q[i] = 0.1 * i; a0.q_p[i] = 0.3; a0.q_pp[i] = -0.2;
        a1.q[i] = 1.0 - 0.1 * i; a1.q_p[i] = -0.1; a1.q_pp[i] = 0.4;
    }
    Polynom pol;
    pol.CalcPolynom(&a0, &a1, 1200);
    const scalar h_ms = 1e-3; // 1 microsecond
    for (int i = 0; i < JOINTS_COUNT; i++)
        for (scalar t = 100; t < 1200; t += 250)
        {
            scalar num_speed = (pol.GetAngle(i, t + h_ms) - pol.GetAngle(i, t - h_ms)) / (2 * h_ms / 1000.0);
            scalar num_acc = (pol.GetSpeed(i, t + h_ms) - pol.GetSpeed(i, t - h_ms)) / (2 * h_ms / 1000.0);
            EXPECT_NEAR(pol.GetSpeed(i, t), num_speed, 1e-5);
            EXPECT_NEAR(pol.GetAcceleration(i, t), num_acc, 1e-4);
        }
}

TEST(Polynom, ZeroDisplacementStaysPut)
{
    Angles a;
    for (int i = 0; i < JOINTS_COUNT; i++)
        a.q[i] = 0.7 - 0.2 * i;
    Polynom pol;
    pol.CalcPolynom(&a, &a, 800);
    for (int i = 0; i < JOINTS_COUNT; i++)
        for (scalar t = 0; t <= 800; t += 100)
        {
            EXPECT_NEAR(pol.GetAngle(i, t), a.q[i], 1e-12);
            EXPECT_NEAR(pol.GetSpeed(i, t), 0, 1e-12);
        }
}
