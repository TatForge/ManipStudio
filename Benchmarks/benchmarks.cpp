#include "benchmarks.h"
#include "solver.h"
#include "robo.h"
#include "benchmark/benchmark.h"
#include "my_math.h"

Benchmarks::Benchmarks()
{

}

//void SolveDirectKinematic(const Angles &q, QMatrix4x4 TR[])
//{
//    const Matrix &A = {{0,     -M_PI/2, 0.2325, q.q[0]},
//                       {0.375,  0,      0,      q.q[1]},
//                       {0.295,  0,      0,      q.q[2]},
//                       {0,     -M_PI/2, 0.1205, q.q[3]},
//                       {0,      M_PI/2, 0.171,  q.q[4]},
//                       {0,      0,      0.1226, q.q[5]}};
//    QMatrix4x4 T[JOINTS_COUNT];
//    T[0] = QMatrix4x4(cos(q.q[0]),         -sin(q.q[0]),       0,           A.at(0,0),
//             sin(q.q[0]) * cos(0), cos(q.q[0]) * cos(0), -sin(0), -sin(0) * A.at(0,2),
//             sin(q.q[0]) * sin(0), cos(q.q[0]) * sin(0),  cos(0),  cos(0) * A.at(0,2),
//                                0,                    0,       0,                  1);
//    for (counter i = 1; i < JOINTS_COUNT; i++)
//    {
//        T[i] = QMatrix4x4(cos(q.q[i]),                                      -sin(q.q[i]),                 0,                   A.at(i-1,0),
//                          sin(q.q[i]) * cos(A.at(i-1,1)), cos(q.q[i]) * cos(A.at(i-1,1)), -sin(A.at(i-1,1)), -sin(A.at(i-1,1)) * A.at(i,2),
//                          sin(q.q[i]) * sin(A.at(i-1,1)), cos(q.q[i]) * sin(A.at(i-1,1)),  cos(A.at(i-1,1)),  cos(A.at(i-1,1)) * A.at(i,2),
//                                                       0,                              0,                 0,                            1);
//    }
//    TR[0] = T[0];
//    TR[1] = T[0] * T[1];
//    TR[2] = T[0] * T[1] * T[2];
//    TR[3] = T[0] * T[1] * T[2] * T[3];
//    TR[4] = T[0] * T[1] * T[2] * T[3] * T[4];
//    TR[5] = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
//}

static void BMDirectKinematic(benchmark::State &state)
{
//    QMatrix4x4 L[JOINTS_COUNT];
//    Angles a;
//    for (auto _ : state)
//    {
//        SolveDirectKinematic(a, L);
//    }
    Angles angles;
    StateVector targets;
    targets.x = -0.35;
    targets.y = -0.3341;
    targets.z = 0.0515;
    targets.roll = ToRad(180);
    Robo manipulator(6);
    Matrix4x4 L[JOINTS_COUNT];
    try {
        manipulator.LoadManip();
        Solver s(manipulator);
        for (auto e : state)
        {
            s.SolveDirectKinematic(angles, L);
        }
    }  catch (std::string &exception) {
        cout << exception << endl;
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
            s.SolveInverseKinematic(targets, angles, angles);
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
    Solver s(manip);
    Angles angles;
    for (auto _ : state)
    {
        s.SolveForwardDynamic(angles);
    }
}
BENCHMARK(BMForwardDynamic)->Unit(benchmark::kMicrosecond);

static void BMCalcUMatrix(benchmark::State &state)
{
    Robo manipulator(6);
    manipulator.LoadManip();
    Solver solver(manipulator);

    for (auto e : state)
    {
        solver.CalcUMatrix();
    }
}
BENCHMARK(BMCalcUMatrix)->Unit(benchmark::kMicrosecond);

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
            solver.MoveL(stv, tarv, 1, 0.5,  start_an, res_an, 10, result);
        }  catch (std::runtime_error &exc_str) {
            cout << exc_str.what() << endl;
        }
    }
}
BENCHMARK(BMCalcPolynomLine)->Unit(benchmark::kMicrosecond);

static void Matrix4x4Mulitplication(benchmark::State &state)
{
    Matrix4x4 m1{1,2,3,4,7,6,5,4,8,7,6,5,2,4,6,7};
    Matrix4x4 m2{8,7,4,5,2,3,8,7,5,4,0,8,6,2,1,4};
    Matrix4x4 m3;
    m3.MakeIdentity();
    for (auto _ : state)
    {
//        for (int i = 0; 10; i++)
//        {
            m3 = m3 * m1 * m2 * m3 * m1 * m2 * m3 * m1 * m2;
//        }
//        m3.SelfMultipl(m1, m2);
    }
}
BENCHMARK(Matrix4x4Mulitplication)->Unit(benchmark::kNanosecond);
//static void QMatrix4x4Mulitplication(benchmark::State &state)
//{
//    QMatrix4x4 m1{1,2,3,4,7,6,5,4,8,7,6,5,2,4,6,7};
//    QMatrix4x4 m2{8,7,4,5,2,3,8,7,5,4,0,8,6,2,1,4};
//    QMatrix4x4 m3;
//    m3.setToIdentity();
//    for (auto _ : state)
//    {
////        for (int i = 0; 10; i++)
////        {
//            m3 = m3 * m1 * m2 * m3 * m1 * m2 * m3 * m1 * m2;
////        }
////        m3.SelfMultipl(m1, m2);
//    }
//}
//BENCHMARK(QMatrix4x4Mulitplication)->Unit(benchmark::kNanosecoкnd);
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
