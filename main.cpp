#include "robo.h"
#include "cuda_math.h"
#include "emulator.h"
#include "solver.h"
#include "meshcollider.h"

#include <main.h>
#include <array>
#include <chrono>
#include <string.h>
#include <thread>
#include <stl_reader.h>

#include "sharedmem.h"

#ifdef WITH_TESTS
#include <QtTest/QTest>
#endif

#if WITH_WINDOW
#include <QApplication>
#endif


//#include "commandworker.h"


using namespace RoboMath;

#if ((!WITH_BENCHMARK) && (!WITH_GTESTS))
int main(int argc, char *argv[])
{
#if WITH_WINDOW
    QApplication app(argc, argv);
#endif
//    SysVSharedMemory shml(FILE_POINTS_SHM, 8196);
//    float f[2048];
//    for (int i = 0; i < 2048; i++)
//    {
//        f[i] = i;
//    }
//    shml.SendData(f, 106);

    Robo manipulator(6);
    try{
        manipulator.LoadManip();
    } catch (string exc_str)
    {
        cout << exc_str << endl;
    }
    rm::Angles angle_stop;
    rm::Angles angle_start;
    RoboMath::Polynom pol;
    for  (int i = 0; i < 6; i++)
    {
        angle_start.q[i] = 0;
        angle_start.q_p[i] = 0;
        angle_start.q_pp[i] = 0;
        angle_stop.q[i] = manipulator.GetTheta(i);
    }

    Solver solver(manipulator);
    Matrix4x4 TT[6];
    Angles start_an;
    Angles end_an;
    start_an.q[0] = 0;
    start_an.q[1] = -M_PI;
    start_an.q[2] = 0;
    start_an.q[3] = -M_PI + M_PI/4;
    start_an.q[4] = 0;
    start_an.q[5] = 0;
//    end_an.q[3] = -M_PI/2;
    end_an.q[3] = 0;
    Angles res_an;
//    start_an = solver.GetExtarnalPosition();
//    start_an.q[1] += 0.2;
//    std::thread thread_dynamic();

    StateVector stv(0.2, -0.2, 0.115, ToRad(100), ToRad(0), ToRad(0));
    StateVector tarv(0.1, 0.241, 0.05, ToRad(180), ToRad(45), ToRad(0));
    StateVector tarv1(-0.2, 0.241, 0.05, ToRad(179), ToRad(45), ToRad(0));
//    StateVector stv(  -0.35, -0.3441, 0.0515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv(  0.35, -0.1341, 0.1515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv1(-0.15, +0.241, 0.0515, ToRad(178), ToRad(-45), ToRad(0));
    StateVector tarv2(-0.35, +0.1341, 0.12, ToRad(178), ToRad(0), ToRad(0));
    StateVector tarv3(-0.35, -0.3341, 0.2515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv4(-0.45, -0.3341, 0.4515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv5( 0.35, -0.1341, 0.1515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv6( 0.15, +0.3341, 0.2515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv7(-0.35, +0.1341, 0.1515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv8(-0.35, -0.3341, 0.2515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv(0.3, -0.2341, 0.1515, ToRad(90), ToRad(+90),   ToRad(+90));

    Polynom result[1000];
//    scalar v_tcp(2.12);
//    scalar a_tcp(4);
    scalar v_tcp(2.2);
    scalar a_tcp(4);

    Angles res_buf;
    Angles a1, a2, a3;
    a1.q[0] = 0.0; a1.q[1] = -M_PI/2; a1.q[2] = +2.6;
    a1.q[3] = -M_PI; a1.q[4] = -M_PI/2;    a1.q[5] = 0.0;
//    a2.q[0] = -1.95; a2.q[1] = -0.55; ar2. q[2] = 1.05;
//    a2.q[3] = 1.43; a2.q[4] = -2.3; a2.q[5] = 1.17;
    scalar bb(0.475);
//    bb = 0;
    a2.q[0] = 0;      a2.q[1] = 0 - bb;        a2.q[2] = 0 + 2*bb;      a2.q[3] = -M_PI/2 - bb;  a2.q[4] = 0;      a2.q[5] = 0;
    a3.q[0] = 0;      a3.q[1] = -M_PI;       a3.q[2] = -M_PI/2;       a3.q[3] = -M_PI/2;       a3.q[4] = 0;      a3.q[5] = 0;
//    auto kk = solver.FindAngle(Vector3(1, 2, 3), Vector3(4, 8, 14), Vector3(8, 10, 1));
//    solver.MoveCircle(Vector3(1, 2,  3), Vector3(2,4,6), Vector3(3, 6, 9), v_tcp, a_tcp, ToSec(10));
    CheckTime ct1;
    vector<float> vr;
    vector<float> t1{0.4, 0.241, 0.1, 180, 45, 0};
    try {
        vector<float> temp;
        temp.resize(18);
        solver.MoveJ(end_an, start_an, v_tcp, a_tcp, 10, vr);
//        solver.MoveL(StateVector(Vector3(0.45, 0.2, 0.11),Vector3(ToRad(180), 0, 0)), StateVector(Vector3(0.35, -0.2, 0.21),Vector3(ToRad(180), 0, 0)), v_tcp, a_tcp, solver.GetExtarnalPosition(), 10, vr);
//        solver.MoveL(StateVector(Vector3(0.35, -0.2, 0.21),Vector3(ToRad(180), 0, 0)), StateVector(Vector3(0.35, -0.2, 0.05),Vector3(ToRad(180), 0, 0)), v_tcp, a_tcp, solver.GetExtarnalPosition(),10, vr);
//        solver.MoveL(StateVector(Vector3(0.35, -0.2, 0.05),Vector3(ToRad(180), 0, 0)), StateVector(Vector3(0.35, -0.2, 0.21),Vector3(ToRad(180), 0, 0)), v_tcp, a_tcp, solver.GetExtarnalPosition(), 10, vr);

//        solver.MoveC(solver.GetExtarnalPosition(),      StateVector(Vector3(0.35, -0.2, 0.21),  Vector3(ToRad(180), 0, 0)),
//                                       StateVector(Vector3(0.4, -0.3, 0.21),  Vector3(ToRad(180), 0, 0)),
//                                       StateVector(Vector3(0.2, -0.3, 0.21),  Vector3(ToRad(180), 0, 0)), v_tcp, a_tcp, 10, vr);
//        solver.MoveL(StateVector(Vector3(0.2, -0.3, 0.21),  Vector3(ToRad(180), 0, 0)), StateVector(Vector3(-0.3, -0.2, 0.21),Vector3(ToRad(180), 0, 0)), v_tcp, a_tcp, solver.GetExtarnalPosition(), 10, vr);
//        solver.MoveL(StateVector(Vector3(-0.3, -0.2, 0.21),  Vector3(ToRad(180), 0, 0)), StateVector(Vector3(-0.3, -0.2, 0.05),Vector3(ToRad(180), 0, 0)), v_tcp, a_tcp, solver.GetExtarnalPosition(), 10, vr);
    }  catch (std::runtime_error &exc) {
        cout << exc.what() << endl;
    }

    cout << " Time = " << ct1.duration().count() << endl;

    cout << "!!!___FINISH___!!!" << endl;
#if WITH_WINDOW
    return app.exec();
#else
    return 0;
#endif
}
#endif
