// Phase 3: Solver dynamics (joint torques tau = D(q) q'' + h(q, q') + G(q)).
//
// The reference values are derived independently from the Lagrangian of the
// same rigid-body model (link masses m_mass, centres of mass m_cm and
// pseudo-inertia matrices m_ji taken from the Solver), using only the
// reference forward kinematics and numerical differentiation:
//   potential  V(q)     = sum_j m_j g z_j(q)            ->  G = dV/dq
//   kinetic    K(q,q')  = 1/2 sum_j Tr(T'_j J_j T'_j^T) ->  D = d2K/dq'2
//   velocity   h        = D' q' - dK/dq                 (Coriolis/centrifugal)

#include "solver_fixture.h"

using namespace solver_test;

namespace {

using Mat6 = std::array<std::array<double, 6>, 6>;

class SolverDynamics : public SolverTest
{
protected:
    // Solver torques for state (q, q', q''). SolveForwardDynamic() builds
    // its kinematics from m_T, which only SolveDirectKinematic() updates, so
    // that is called first (see UsesTheAnglesItIsGiven for why this matters).
    Robo::Moments Torques(const Angles &state)
    {
        Matrix4x4 TR[JOINTS_COUNT];
        solver->SolveDirectKinematic(state, TR);
        try
        {
            solver->SolveForwardDynamic(state);
        }
        catch (const std::runtime_error &)
        {
            // CheckMaxMoments() throws *after* the moments were computed.
        }
        return solver->m_moments;
    }

    Mat4 FromMatrix4x4(const Matrix4x4 &m)
    {
        Mat4 r;
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                r[i][k] = m.at(i, k);
        return r;
    }

    // dT0j/dq_k by central differences of the reference FK.
    Mat4 dT(const double q[6], int j, int k)
    {
        const double h = 1e-6;
        double qp[6], qm[6];
        std::copy(q, q + 6, qp);
        std::copy(q, q + 6, qm);
        qp[k] += h;
        qm[k] -= h;
        Mat4 a = ReferenceT0(qp, j), b = ReferenceT0(qm, j), r;
        for (int r1 = 0; r1 < 4; r1++)
            for (int c = 0; c < 4; c++)
                r[r1][c] = (a[r1][c] - b[r1][c]) / (2 * h);
        return r;
    }

    static double TraceABCt(const Mat4 &A, const Mat4 &B, const Mat4 &C)
    {
        // Tr(A * B * C^T)
        double t = 0;
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                for (int l = 0; l < 4; l++)
                    t += A[i][k] * B[k][l] * C[i][l];
        return t;
    }

    double Potential(const double q[6])
    {
        double v = 0;
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            Mat4 t = ReferenceT0(q, j);
            const Vector &cm = solver->m_cm[j];
            double z = t[2][0] * cm.at(0) + t[2][1] * cm.at(1) + t[2][2] * cm.at(2) + t[2][3] * cm.at(3);
            v += solver->m_mass[j] * g * z;
        }
        return v;
    }

    Mat6 ReferenceInertia(const double q[6])
    {
        Mat6 D{};
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            Mat4 J = FromMatrix4x4(solver->m_ji[j]);
            Mat4 d[6];
            for (int k = 0; k <= j; k++)
                d[k] = dT(q, j, k);
            for (int i = 0; i <= j; i++)
                for (int k = 0; k <= j; k++)
                    D[i][k] += TraceABCt(d[i], J, d[k]);
        }
        return D;
    }

    std::array<double, 6> ReferenceVelocityTerm(const double q[6], const double qd[6])
    {
        const double e = 1e-5;
        double qp[6], qm[6];
        for (int i = 0; i < 6; i++)
        {
            qp[i] = q[i] + e * qd[i];
            qm[i] = q[i] - e * qd[i];
        }
        Mat6 Dp = ReferenceInertia(qp), Dm = ReferenceInertia(qm);
        auto K = [&](const double *qq) {
            Mat6 D = ReferenceInertia(qq);
            double k = 0;
            for (int i = 0; i < 6; i++)
                for (int l = 0; l < 6; l++)
                    k += 0.5 * qd[i] * D[i][l] * qd[l];
            return k;
        };
        std::array<double, 6> h{};
        for (int i = 0; i < 6; i++)
        {
            double ddot_qd = 0;
            for (int l = 0; l < 6; l++)
                ddot_qd += (Dp[i][l] - Dm[i][l]) / (2 * e) * qd[l];
            double a[6], b[6];
            std::copy(q, q + 6, a);
            std::copy(q, q + 6, b);
            a[i] += e;
            b[i] -= e;
            h[i] = ddot_qd - (K(a) - K(b)) / (2 * e);
        }
        return h;
    }

    // Column k of the Solver's inertia matrix: dv for q' = 0, q'' = e_k.
    Mat6 SolverInertia(const Angles &pose)
    {
        Mat6 D{};
        for (int k = 0; k < JOINTS_COUNT; k++)
        {
            Angles s = pose;
            for (int j = 0; j < JOINTS_COUNT; j++)
                s.q_p[j] = s.q_pp[j] = 0;
            s.q_pp[k] = 1;
            Robo::Moments m = Torques(s);
            for (int i = 0; i < JOINTS_COUNT; i++)
                D[i][k] = m.dv[i];
        }
        return D;
    }
};

const std::vector<Angles> &DynPoses()
{
    static const std::vector<Angles> poses{WorkPose(), MakeAngles({0, -M_PI / 4, M_PI / 2, M_PI / 4, M_PI / 2, 0}),
                                           MakeAngles({-0.4, -0.6, 1.2, -0.3, 0.7, 0.5}), MakeAngles({1.1, -0.3, 0.8, 0.6, -0.9, -0.4})};
    return poses;
}

} // namespace

TEST_F(SolverDynamics, AtRestTorqueIsPureGravity)
{
    for (const auto &q : DynPoses())
    {
        Robo::Moments m = Torques(q); // q_p = q_pp = 0
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            EXPECT_NEAR(m.dv[i], 0, 1e-12);
            EXPECT_NEAR(m.hikm[i], 0, 1e-12);
            EXPECT_NEAR(m.tao[i], m.p[i], 1e-12);
        }
    }
}

TEST_F(SolverDynamics, GravityTorqueIsGradientOfPotentialEnergy)
{
    const double h = 1e-6;
    for (const auto &q : DynPoses())
    {
        Robo::Moments m = Torques(q);
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            double qp[6], qm[6];
            std::copy(q.q, q.q + 6, qp);
            std::copy(q.q, q.q + 6, qm);
            qp[i] += h;
            qm[i] -= h;
            double expected = (Potential(qp) - Potential(qm)) / (2 * h);
            EXPECT_NEAR(m.p[i], expected, 1e-5 + 1e-6 * std::abs(expected)) << "joint " << i;
        }
    }
}

TEST_F(SolverDynamics, BaseJointFeelsNoGravity)
{
    for (const auto &q : DynPoses())
        EXPECT_NEAR(Torques(q).p[0], 0, 1e-9);
}

// Holding a heavier arm pose must cost more: stretching the shoulder out
// horizontally increases the gravity torque of joint 1.
TEST_F(SolverDynamics, ShoulderGravityTorqueGrowsWithLeverArm)
{
    Angles folded = MakeAngles({0, -M_PI / 2 + 0.1, 2.0, 0, 0.5, 0});
    Angles stretched = MakeAngles({0, 0, 0.1, 0, 0.5, 0});
    EXPECT_GT(std::abs(Torques(stretched).p[1]), std::abs(Torques(folded).p[1]));
}

TEST_F(SolverDynamics, InertialTermIsLinearInAcceleration)
{
    Angles a = WorkPose(), b = WorkPose(), ab = WorkPose();
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        a.q_pp[j] = 0.3 * (j + 1);
        b.q_pp[j] = -0.2 + 0.1 * j;
        ab.q_pp[j] = 2 * a.q_pp[j] - 3 * b.q_pp[j];
    }
    Robo::Moments ma = Torques(a), mb = Torques(b), mab = Torques(ab);
    for (int i = 0; i < JOINTS_COUNT; i++)
        EXPECT_NEAR(mab.dv[i], 2 * ma.dv[i] - 3 * mb.dv[i], 1e-9);
}

// Regression test for a fixed bug: the D term uses the pseudo-inertia of link k
// (m_ji[k]) instead of link j (m_ji[j]) inside the sum over j, which makes
// the inertia matrix non-symmetric (a symmetric D is a fundamental property
// of any mechanical system).
TEST_F(SolverDynamics, InertiaMatrixIsSymmetric)
{
    for (const auto &q : DynPoses())
    {
        Mat6 D = SolverInertia(q);
        for (int i = 0; i < 6; i++)
            for (int k = i + 1; k < 6; k++)
                EXPECT_NEAR(D[i][k], D[k][i], 1e-9 + 1e-6 * std::abs(D[i][k])) << "D(" << i << "," << k << ")";
    }
}

TEST_F(SolverDynamics, InertiaMatrixMatchesKineticEnergyReference)
{
    for (const auto &q : DynPoses())
    {
        Mat6 D = SolverInertia(q), R = ReferenceInertia(q.q);
        for (int i = 0; i < 6; i++)
            for (int k = 0; k < 6; k++)
                EXPECT_NEAR(D[i][k], R[i][k], 1e-6 + 1e-5 * std::abs(R[i][k])) << "D(" << i << "," << k << ")";
    }
}

// Diagnostic companion of the two tests above: the same D assembled from
// the Solver's *own* derivative matrices U (CalcUMatrix) but with the link
// index fixed (m_ji[j]) matches the kinetic-energy reference. This both
// validates the reference and pins the defect to the m_ji index alone.
TEST_F(SolverDynamics, SolverDerivativeMatricesGiveReferenceInertiaWithCorrectLinkIndex)
{
    for (const auto &q : DynPoses())
    {
        Matrix4x4 TR[JOINTS_COUNT];
        solver->SolveDirectKinematic(q, TR);
        solver->CalcUMatrix();
        Mat6 R = ReferenceInertia(q.q);
        for (int i = 0; i < 6; i++)
            for (int k = 0; k < 6; k++)
            {
                double d = 0;
                for (int j = std::max(i, k); j < 6; j++)
                    d += Trace(solver->U[k * 6 + j] * solver->m_ji[j].MultiplyTranspose(solver->U[i * 6 + j]));
                EXPECT_NEAR(d, R[i][k], 1e-6 + 1e-5 * std::abs(R[i][k])) << "D(" << i << "," << k << ")";
            }
    }
}

TEST_F(SolverDynamics, InertiaMatrixIsPositiveDefinite)
{
    for (const auto &q : DynPoses())
    {
        Mat6 D = SolverInertia(q);
        for (int trial = 0; trial < 20; trial++)
        {
            double x[6];
            for (int i = 0; i < 6; i++)
                x[i] = std::sin(1.3 * trial + 0.7 * i) + (i == trial % 6 ? 1.0 : 0.0);
            double quad = 0;
            for (int i = 0; i < 6; i++)
                for (int k = 0; k < 6; k++)
                    quad += x[i] * D[i][k] * x[k];
            EXPECT_GT(quad, 0) << "trial " << trial;
        }
    }
}

TEST_F(SolverDynamics, VelocityTermVanishesAtRestAndIsQuadraticInSpeed)
{
    Angles s1 = WorkPose(), s2 = WorkPose();
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        s1.q_p[j] = 0.2 - 0.07 * j;
        s2.q_p[j] = 2 * s1.q_p[j];
    }
    Robo::Moments m1 = Torques(s1), m2 = Torques(s2);
    for (int i = 0; i < JOINTS_COUNT; i++)
        EXPECT_NEAR(m2.hikm[i], 4 * m1.hikm[i], 1e-9 + 1e-9 * std::abs(m1.hikm[i])) << "joint " << i;
}

// Regression test for fixed bugs in the h (Coriolis/centrifugal) term: it uses the
// pseudo-inertia of link i instead of link j, builds the second derivative
// of T0j with the classic-DH form (Q * T) while the kinematics are modified
// DH (T * Q, as CalcUMatrix correctly does), and stops the product one
// transform early (ctr < j).
TEST_F(SolverDynamics, VelocityTermMatchesLagrangianReference)
{
    for (const auto &q : DynPoses())
    {
        Angles s = q;
        for (int j = 0; j < JOINTS_COUNT; j++)
            s.q_p[j] = 0.5 - 0.15 * j;
        Robo::Moments m = Torques(s);
        auto ref = ReferenceVelocityTerm(s.q, s.q_p);
        for (int i = 0; i < JOINTS_COUNT; i++)
            EXPECT_NEAR(m.hikm[i], ref[i], 1e-4 + 1e-3 * std::abs(ref[i])) << "joint " << i;
    }
}

// Regression test for a fixed design bug: SolveForwardDynamic(angles) takes the joint
// *positions* from m_T -- whatever the last SolveDirectKinematic() call left
// there -- and only uses `angles` for q' and q''. Asking for the torques of
// pose B right after a kinematics call for pose A silently returns pose A's
// torques.
TEST_F(SolverDynamics, UsesTheAnglesItIsGiven)
{
    Angles a = DynPoses()[0], b = DynPoses()[3];
    Robo::Moments expected = Torques(b); // FK(b) + dynamics(b)
    Matrix4x4 TR[JOINTS_COUNT];
    solver->SolveDirectKinematic(a, TR); // unrelated kinematics call
    try
    {
        solver->SolveForwardDynamic(b);
    }
    catch (const std::runtime_error &)
    {
    }
    for (int i = 0; i < JOINTS_COUNT; i++)
        EXPECT_NEAR(solver->m_moments.tao[i], expected.tao[i], 1e-9) << "joint " << i;
}

TEST_F(SolverDynamics, TotalTorqueIsSumOfTerms)
{
    Angles s = DynPoses()[2];
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        s.q_p[j] = 0.3 - 0.1 * j;
        s.q_pp[j] = -0.5 + 0.2 * j;
    }
    Robo::Moments m = Torques(s);
    for (int i = 0; i < JOINTS_COUNT; i++)
        EXPECT_NEAR(m.tao[i], m.p[i] + m.dv[i] + m.hikm[i], 1e-12) << "joint " << i;
}

TEST_F(SolverDynamics, CheckMaxMomentsBoundary)
{
    Robo::Moments m = Torques(WorkPose());
    solver->m_moments = m;
    auto saved = solver->max_joint_moments;
    for (int i = 0; i < JOINTS_COUNT; i++)
        solver->max_joint_moments[i] = std::abs(m.tao[i]); // exactly at the limit: allowed
    EXPECT_NO_THROW(solver->CheckMaxMoments());
    solver->max_joint_moments[1] = std::abs(m.tao[1]) * 0.999; // just above the limit
    EXPECT_THROW(solver->CheckMaxMoments(), std::runtime_error);
    solver->max_joint_moments = saved;
}

TEST_F(SolverDynamics, CheckMaxMomentsThrowsWhenLimitExceeded)
{
    Angles s = WorkPose();
    for (int j = 0; j < JOINTS_COUNT; j++)
        s.q_pp[j] = 500; // absurd acceleration
    Matrix4x4 TR[JOINTS_COUNT];
    solver->SolveDirectKinematic(s, TR);
    EXPECT_THROW(solver->SolveForwardDynamic(s), std::runtime_error);
}

TEST_F(SolverDynamics, HoldingWorkPoseStaysWithinMotorLimits)
{
    Matrix4x4 TR[JOINTS_COUNT];
    solver->SolveDirectKinematic(WorkPose(), TR);
    EXPECT_NO_THROW(solver->SolveForwardDynamic(WorkPose()));
    for (int i = 0; i < JOINTS_COUNT; i++)
        EXPECT_LE(std::abs(solver->m_moments.tao[i]), solver->max_joint_moments[i]);
}

TEST_F(SolverDynamics, MaxMomentsComeFromSettingsFile)
{
    ASSERT_EQ(solver->max_joint_moments.size(), 6u);
    EXPECT_EQ(solver->max_joint_moments, robot->GetMaxJointMoments());
}

// Trajectory level: every sample of a MoveJ gets a dynamics evaluation, and
// the recorded moments are the torques for that sample's state.
TEST_F(SolverDynamics, MoveJEvaluatesDynamicsForEverySample)
{
    Angles to = WorkPose();
    to.q[1] -= 0.4;
    std::vector<float> flat;
    ASSERT_TRUE(RunsWithoutThrowing([&] { solver->MoveJ(WorkPose(), to, 1.0f, 2.0f, 10.0f, flat); }));
    const auto samples = Samples(flat);
    ASSERT_EQ(solver->m_moments_counter, samples.size());
    auto recorded = solver->m_moments_v; // copy before Torques() overwrites state
    auto fresh = NewSolver();
    for (size_t k = 0; k < samples.size(); k += 7)
    {
        Matrix4x4 TR[JOINTS_COUNT];
        fresh->SolveDirectKinematic(samples[k], TR);
        try
        {
            fresh->SolveForwardDynamic(samples[k]);
        }
        catch (const std::runtime_error &)
        {
        }
        for (int i = 0; i < JOINTS_COUNT; i++)
            EXPECT_NEAR(recorded[k].tao[i], fresh->m_moments.tao[i], 1e-3 + 1e-4 * std::abs(fresh->m_moments.tao[i]))
                << "sample " << k << " joint " << i;
    }
}
