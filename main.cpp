#include "Tests/test_my_math.h"
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
#include <QSharedMemory>
#include <stl_reader.h>


#ifdef WITH_TESTS
#include <QtTest/QTest>
#endif

#if WITH_WINDOW
#include <QApplication>
#endif

#include <QString>
QString a11 = QString::number(123);

//#include "commandworker.h"


using namespace RoboMath;

#if ((!WITH_BENCHMARK) && (!WITH_GTESTS))
int main(int argc, char *argv[])
{
#if WITH_WINDOW
    QApplication app(argc, argv);
#endif
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
    Angles res_an;

//    std::thread thread_dynamic();

    StateVector stv( 0.2, -0.2, 0.115, ToRad(100), ToRad(0), ToRad(0));
    StateVector tarv(0.1, 0.241, 0.05, ToRad(179), ToRad(45), ToRad(0));
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
    scalar v_tcp(0.2);
    scalar a_tcp(1);
    CheckTime ct1;
    try {
        solver.CalcPolynomLine(stv,   tarv,  v_tcp, a_tcp, angle_start, res_an, 10, result);
//        solver.CalcPolynomLine(tarv,  tarv1, v_tcp, a_tcp, res_an,      res_an, 10, result);
//        solver.CalcPolynomLine(tarv1, tarv2, v_tcp, a_tcp, res_an,      res_an, 10, result);
//        solver.CalcPolynomLine(tarv2, tarv3, v_tcp, a_tcp, res_an,      res_an, 10, result);
//        solver.CalcPolynomLine(tarv3, tarv4, v_tcp, a_tcp, res_an,      res_an, 10, result);
    }  catch (std::runtime_error &exc) {
        cout << exc.what() << endl;
    }

    cout << " Time = " << ct1.duration().count() << endl;

#ifdef WITH_TESTS
    QTest::qExec(new test_my_math, argc, argv);
#endif
    cout << "!!!___FINISH___!!!" << endl;
#if WITH_WINDOW
    return app.exec();
#else
    return 0;
#endif
}
#endif
