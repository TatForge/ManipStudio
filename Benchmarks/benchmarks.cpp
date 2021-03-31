#include "benchmarks.h"
#include "solver.h"
#include "robo.h"
#include "benchmark/benchmark.h"
#include "my_math.h"

Benchmarks::Benchmarks()
{

}

void SolveDirectKinematic(const Angles &q, Matrix4x4 TR[])
{
    const Matrix &A = {{0,     -M_PI/2, 0.2325, q.q[0]},
                       {0.375,  0,      0,      q.q[1]},
                       {0.295,  0,      0,      q.q[2]},
                       {0,     -M_PI/2, 0.1205, q.q[3]},
                       {0,      M_PI/2, 0.171,  q.q[4]},
                       {0,      0,      0.1226, q.q[5]}};
    Matrix4x4 T[JOINTS_COUNT];
    T[0] = {{         cos(q.q[0]),         -sin(q.q[0]),       0,           A.at(0,0)},
            {sin(q.q[0]) * cos(0), cos(q.q[0]) * cos(0), -sin(0), -sin(0) * A.at(0,2)},
            {sin(q.q[0]) * sin(0), cos(q.q[0]) * sin(0),  cos(0),  cos(0) * A.at(0,2)},
            {                   0,                    0,       0,                   1}};
    for (counter i = 1; i < JOINTS_COUNT; i++)
    {
        T[i] = {{cos(q.q[i]),                                      -sin(q.q[i]),                 0,                   A.at(i-1,0)},
                {sin(q.q[i]) * cos(A.at(i-1,1)), cos(q.q[i]) * cos(A.at(i-1,1)), -sin(A.at(i-1,1)), -sin(A.at(i-1,1)) * A.at(i,2)},
                {sin(q.q[i]) * sin(A.at(i-1,1)), cos(q.q[i]) * sin(A.at(i-1,1)),  cos(A.at(i-1,1)),  cos(A.at(i-1,1)) * A.at(i,2)},
                {                                0,                                 0,                 0,                             1}};
    }
    TR[0] = T[0];
    TR[1] = T[0] * T[1];
    TR[2] = T[0] * T[1] * T[2];
    TR[3] = T[0] * T[1] * T[2] * T[3];
    TR[4] = T[0] * T[1] * T[2] * T[3] * T[4];
    TR[5] = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
}

//void InverseKinematic(const StateVector &target, const Angles &curr_angle)
//{
//    Quaternion end_eff_target_ori;
//    Quaternion end_eff_world_ori;
//    Quaternion deltaQ;
//    Matrix4x4 TR[JOINTS_COUNT];
//    Vector3 ang_vel;
//    Vector3 delta_r;
//    Vector3 delta_s;
//    Vector3 axis;
//    scalar accuracy = 10e-5;
////    StateVector target(0.35, -0.1341, 0.2515, ToRad(180), ToRad(0), ToRad(0));
//    Matrix jacobian(EFFECTOR_DEGREE_OF_FREEDOM,6);
//    Matrix jacobian_transposed(EFFECTOR_DEGREE_OF_FREEDOM, 6);
//    Matrix U(EFFECTOR_DEGREE_OF_FREEDOM, EFFECTOR_DEGREE_OF_FREEDOM);
//    Matrix aug_matrix(U.GetRowsCount(), U.GetColsCount() + 1);
//    Angles d_thet;
//    for (int i = 0; i < 6; i++)
//        d_thet.q[i] = i/6;
//    Vector delta_t(EFFECTOR_DEGREE_OF_FREEDOM);
//    Vector delta_thetas(JOINTS_COUNT);
//    counter ctr(0);
//    scalar angle(0);

//    bool solved = false;
//    SolveDirectKinematic(d_thet, TR);

//    end_eff_world_ori = TR[JOINTS_COUNT - 1].ConvertToQuaternion();
//    end_eff_target_ori.EulerToQuat(target.GetAngles());
//    end_eff_world_ori.Invert();
//    deltaQ = end_eff_target_ori * end_eff_world_ori;
//    angle = deltaQ.GetAngle();
//    axis = deltaQ.GetAxis();
//    if (angle > M_PI)
//        angle -= 2.0 * M_PI;
//    else if (angle < - M_PI)
//        angle += 2.0 * M_PI;
//    ang_vel = axis.Normalize() * angle;
//    delta_r = ang_vel * 1;
//    delta_s.x = target.x - TR[5](0,3);
//    delta_s.y = target.y - TR[5](1,3);
//    delta_s.z = target.z - TR[5](2,3);
////    auto kk = delta_s.Length() > accuracy;
////    auto kk1 = delta_r.Length() > accuracy * 100;
//    while ((delta_s.Length() > accuracy) || (delta_r.Length() > accuracy * 100) || !solved)
//    {
////        cout << "delta_r " << delta_r.Length() << " delta_s " << delta_s.Length() << endl;
//        solved = true;
//        for (int i = 0; i < JOINTS_COUNT; i++)
//        {
//            jacobian.SetSubColumn(Vector3(TR[i](0,2), TR[i](1,2), TR[i](2,2)) * (Vector3({TR[5](0,3), TR[5](1,3), TR[5](2,3)}) - Vector3({TR[i](0,3), TR[i](1,3), TR[i](2,3)})), i, 0);
//#ifdef POSITION_AND_ROTATION
//            jacobian.SetSubColumn(Vector3(TR[i](0,2), TR[i](1,2), TR[i](2,2)), i, 3);
//#endif
//        }
//        //calc Jacobian for current configuration
//        jacobian_transposed = jacobian;
////        jacobian.MatrixCout("jacobian");
//        jacobian_transposed.Transpose();
////        jacobian_transposed.MatrixCout("jacobian_tra");
//        U = (jacobian) * jacobian_transposed;
//        U.AddDiagonal(damping_constant_sq);
////        U.MatrixCout("U");
//        aug_matrix.SetSubmatrix(U, 0, 0);
//        aug_matrix.SetSubColumn(delta_s, aug_matrix.GetColsCount() - 1, 0);
////        aug_matrix.MatrixCout("Matrix aug");
//#ifdef POSITION_AND_ROTATION
//        aug_matrix.SetSubColumn(delta_r, aug_matrix.GetColsCount() - 1, 3);
//#endif
//        aug_matrix.SolveGauss();
//        aug_matrix.GetTheRoots(delta_t);
//        delta_thetas = jacobian_transposed * delta_t;

//        scalar max = delta_thetas.MaxAbs();
//        if (max > MaxAngleDls)
//        {
//            delta_thetas *= MaxAngleDls/max;
//        }
//        for (int i = 0; i < JOINTS_COUNT; i++)
//            d_thet.q[i] += delta_thetas(i);

//        SolveDirectKinematic(d_thet, TR);

//        end_eff_world_ori = TR[5].ConvertToQuaternion();
//        end_eff_target_ori.EulerToQuat(target.GetAngles());
//        end_eff_world_ori.Invert();
//        deltaQ = end_eff_target_ori * end_eff_world_ori;
//        angle = deltaQ.GetAngle();
//        axis = deltaQ.GetAxis();
//        if (angle > M_PI)
//            angle -= 2.0 * M_PI;
//        else if (angle < - M_PI)
//            angle += 2.0 * M_PI;
//        ang_vel = axis.Normalize() * angle;
//        delta_r = ang_vel * 1;
//        delta_s.x = target.x - TR[5](0,3);
//        delta_s.y = target.y - TR[5](1,3);
//        delta_s.z = target.z - TR[5](2,3);
//        ctr++;
//        if (ctr > MAX_KINEMATIC_ITERS)
//        {
//            solved = false;
//            cout << "Точка " << "x(" << target.x << ")" << " y(" << target.y << ")" << " z(" << target.z << ") недостижима" << endl;
//            break;
//        }
//        //не имеет решения
//    }
////    if (solved)
////        for (int i = 0; i < JOINTS_COUNT; i++)
////            res_angle.q[i] = d_thet.q[i];
////    Matrix4x4 TRR[JOINTS_COUNT];
////    SolveDirectKinematic(res_angle, TRR);
////    cout << "InverseKinematic result \nx = " << TRR[5](0,3) << " y = " << TRR[5](1,3) << " z = " << TRR[5](2,3)
////         << "\nroll = " << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().x)
////         << " pitch = " << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().y)
////         << " yaw = "   << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().z)
////         << endl;
////    return 0;
//}
static void BMDirectKinematic(benchmark::State &state)
{
    Matrix4x4 L[JOINTS_COUNT];
    Angles a;
    for (auto _ : state)
    {
        SolveDirectKinematic(a, L);
    }
}
BENCHMARK(BMDirectKinematic)->Unit(benchmark::kMicrosecond);

static void BMInverseKinematic(benchmark::State &state)
{
    Angles angles;
    StateVector targets;
    targets.x = -0.35;
    targets.y = -0.3341;
    targets.z = 0.0515;
    targets.roll = ToRad(180);
    Robo manipulator(6);
    try {
        manipulator.LoadManip();
        Solver s(manipulator);
        for (auto e : state)
        {
            s.SolveInverseKinematic(10e-5, targets, angles, angles);
        }
    }  catch (std::string &exception) {
        cout << exception << endl;
    }
}
BENCHMARK(BMInverseKinematic)->Unit(benchmark::kMicrosecond);

static void BMForwardDynamic(benchmark::State &state)
{
 Robo manip(6);
 manip.LoadManip();
 for (auto _ : state)
 {
     manip.SolveForwardDynamic();
 }
}
BENCHMARK(BMForwardDynamic)->Unit(benchmark::kMicrosecond);

static void BMCalcPolynomLine(benchmark::State &state)
{
    Angles angles;
    StateVector targets;
    StateVector stv( 0.2, -0.2, 0.115, ToRad(100), ToRad(0), ToRad(0));
    StateVector tarv(0.1, 0.2, 0.05, ToRad(179), ToRad(45), ToRad(0));
    Angles start_an;
    Angles res_an;
    Polynom result[1000];
    Robo manipulator(6);
    manipulator.LoadManip();
    Solver solver(manipulator);
    for (auto e : state)
    {
        try {
            solver.CalcPolynomLine(stv, tarv, 1, 0.5,  start_an, res_an, 10, result);
        }  catch (std::runtime_error &exc_str) {
            cout << exc_str.what() << endl;
        }
    }
}
BENCHMARK(BMCalcPolynomLine)->Unit(benchmark::kMicrosecond);

//static void BMCollisionDetection(benchmark::State &state)
//{
//#define WITH_COLLISIONS
//    Angles angles;
//    StateVector targets;
//    StateVector stv( 0.2, -0.2, 0.115, ToRad(100), ToRad(0), ToRad(0));
//    StateVector tarv(0.1, 0.2, 0.05, ToRad(179), ToRad(45), ToRad(0));
//    Angles start_an;
//    Angles res_an;
//    Polynom result[1000];
//    Robo manipulator(6);
//    manipulator.LoadManip();
//    Solver solver(manipulator);
//    for (auto _ : state)
//    {
//        solver.CalcPolynomLine(stv, tarv, start_an, res_an, 10, result);
//    }
//}
//BENCHMARK(BMCollisionDetection)->Unit(benchmark::kMicrosecond);

//static void BM

#if WITH_BENCHMARK
    BENCHMARK_MAIN();
#endif
