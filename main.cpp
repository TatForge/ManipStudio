#include "Tests/test_my_math.h"
#include "robo.h"
#include "cuda_math.h"
#include "emulator.h"
#include "solver.h"
#include "meshcollider.h"
//#include "gjkcollision3d.h"

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

#include <QApplication>

#include <QString>
QString a11 = QString::number(123);

//#include "commandworker.h"


using namespace RoboMath;

#if ((!WITH_BENCHMARK) && (!WITH_GTESTS))
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    array<scalar, 6> tet {0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          90 * M_PI/180,
                          0 * M_PI/180};
    Vector speed{1,1,1,1,1,1};
    Vector acceleration{1,1,1,1,1,1};
//    Vector speed{0,1,0,0,0,0};
//    Vector acceleration{0,0,0,0,0,0};

    Matrix A {{0,     -M_PI/2, 0.2325, tet[0]},
              {0.375,  0,      0,  tet[1]},
              {0.295,  0,      0,  tet[2]},
              {0,     -M_PI/2, 0.1205, tet[3]},
              {0,      M_PI/2, 0.171,  tet[4]},
              {0,      0,      0.1226, tet[5]}};

    array<Matrix4x4, 6> T;

    T[0] = {{cos(A(0,3)),         -sin(A(0,3)),           0,       A(0,0)},
            {sin(A(0,3)) * cos(0), cos(A(0,3)) * cos(0), -sin(0), -sin(0) * A(0,2)},
            {sin(A(0,3)) * sin(0), cos(A(0,3)) * sin(0),  cos(0),  cos(0) * A(0,2)},
            {0,                    0,                     0,       1}};
//    T[0].MatrixCout("T");
    for (counter i = 1; i < T.size(); i++)
    {
        T[i] = {{cos(A(i,3)),                 -sin(A(i,3)),                0,              A(i-1,0)},
                {sin(A(i,3)) * cos(A(i-1,1)), cos(A(i,3)) * cos(A(i-1,1)), -sin(A(i-1,1)), -sin(A(i-1,1)) * A(i,2)},
                {sin(A(i,3)) * sin(A(i-1,1)), cos(A(i,3)) * sin(A(i-1,1)), cos(A(i-1,1)),  cos(A(i-1,1)) * A(i,2)},
                {0,                           0,                           0,              1}};
//        T[i].MatrixCout("T");
    }

//    Matrix4x4 L =  T[0] * T[1] * T[2] * T[3] * T[4]* T[5];
//    cout << " x = " << L(0,3) << " y = " << L(1,3) << " z = " << L(2,3) << endl;

            //Ixx  Iyy  Izz  Ixy   Ixz
    array<Tensor,6> I;
    I[0] = {0.000588, 0.000587, 0.000929, -1.299e-8, 2.44e-8, 1.79e-6};
    I[1] = {0.000928, 0.001512, 0.000937, 1.86e-5,  1.55e-10, 5.34e-12};
    I[2] = {0.001759, 0.000747, 0.001913, 2.32e-8,  6.37e-9,  1.68e-6};
    I[3] = {0.000198, 0.000479, 0.002059, 3.16e-11, 5.59e-10, 1.59e-7};
    I[4] = {9.826e-5, 0.000128, 0.000114, -2.74e-9, 4.41e-9,  5.82e-7};
    I[5] = {5.529e-5, 5.359e-5, 8.415e-5, 1.99e-10, 2.53e-9,  1.85e-7};

    //Массы элементов
    Vector m {0.5982,
              1.1495,
              0.6399,
              0.3551,
              0.1747,
              0.1667};
    Vector gv{0, 0, -g, 0};
    array<Matrix, 6> dAj;
    Matrix Uji(4, 4);
    Uji.MakeIdentity();
    MatrixOfMatrix6x6 U;
    Vector res_hikm(6,0);
//    scalar hikm_buff;
    array<Matrix, 6> Ji;
    Vector Dv(6, 0);
    Vector p(6,0);
    Vector hikm(6,0);
    Matrix u_buff(4,4);
    Vector tao(6,0);

    scalar *T_ptr = new scalar [6 * 4 * 4];
    scalar *Ji_ptr = new scalar [6 * 4 * 4];
    scalar *U_ptr = new scalar [6 * 6 * 4 * 4];
    scalar *speed_ptr = new scalar[6];
//    for (int i = 0; i < 6; i++)
//    {
//        memcpy(T_ptr + i * 4 * 4, T[i].GetDataPtr(), sizeof(scalar) * 4 * 4);
//    }
//    for (int i = 0; i < 6; i++)
//    {
//        for (int j = 0; j < 6; j++)
//            memcpy(U_ptr + i * 6 * 4 * 4 + j * 4 * 4, U(i,j).GetDataPtr(), sizeof(scalar) * 4 * 4);
//    }
//    for (int i = 0; i < 6; i++)
//    {
//        memcpy(Ji_ptr + i * 4 * 4, Ji[i].GetDataPtr(), sizeof(scalar) * 4 * 4);
//    }
//    memcpy(speed_ptr, speed.GetDataPtr(), sizeof(scalar) * 6);

//    for (int i = 0; i < 10; i++)
//    {
//        using namespace std::chrono;
//        auto start = high_resolution_clock::now();
//        CudaCalcucaleHikm(T_ptr, U_ptr, Ji_ptr, speed_ptr, 0);

//        auto stop = high_resolution_clock::now();
//        auto duration = duration_cast<microseconds>(stop - start);

////        auto bf = kktt.duration().count();
//        cout << "Cuda Hikm calculated for i(0) time = " << duration.count() << " microsec" << endl;
//    }



    for (counter i = 0; i < 6; i++)
    {
        tao(i) = Dv(i) + hikm(i) + p(i);
    }
//    tao.VectorCout("TAO");



    Robo manipulator(6);
    try{
        manipulator.LoadManip();
    } catch (string exc_str)
    {
        cout << exc_str << endl;
    }

//    try{
//        manipulator.LoadManip();
//    } catch (string exc_str)
//    {
//        cout << exc_str << endl;
//        initializer_list<Elem> il_elem {Elem(Vector3(0,     0,      0),      unit_z,  ToRad(0)),
//                                        Elem(Vector3(0,     0,      0.2325), unit_y,  ToRad(0)),
//                                        Elem(Vector3(0.375, 0,      0.2325), unit_y,  ToRad(0)),
//                                        Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  ToRad(0)),
//                                        Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, ToRad(0)),
//                                        Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  ToRad(0))};
//        manipulator = Robo(il_elem);
//        manipulator.WriteSettingsToFile();
//    }

//    manipulator.ComputeJointsPosition();
//    manipulator.SolveForwardDynamic();
//    manipulator.WriteSettingsToFile();
//    manipulator.SolveInverseDynamic();
//    manipulator.SolveDirectKinematic(manipulator.get)
//    Robo::Moments *res_mom = manipulator.GetMoments();
//    rm::Angles *res_ang = manipulator.GetAngles();
//    for (int i = 0; i < 6; i++)
//    {
//        cout << "tao " << setw(10) << res_mom->tao[i]
//             << "\tDv "  << setw(10) << res_mom->dv[i]
//             << "\thikm "<< setw(10) << res_mom->hikm[i]
//             << "\tp "   << setw(10) << res_mom->p[i]
//             << endl;
//    }
//    for (int i = 0; i < 6; i++)
//    {
//        cout << "Q " << setw(10) << res_ang->q[i]
//             << "\tQp " << setw(10) << res_ang->q_p[i]
//             << "\tQpp " << setw(10) << res_ang->q_pp[i]
//             << endl;
//    }
//    cout << "Duration " << tim.duration().count() << endl;
//    hikm.VectorCout("hikm");
//    Dv.VectorCout("Dv");
//    p.VectorCout("p");
//    Vector3 target(-0.114, -0.2501, 0.1355);
//    Vector3 target_r(ToRad(180), ToRad(0),   ToRad(0));
//    int kolvo = 1;
//    CheckTime ct;
//    for (int i = 0; i < kolvo; i ++)
//    {
//        manipulator.SolveInverseKinematic(0.00001, target, target_r);
//    }
//    cout << "Time of " << kolvo << " calculations " << ct.duration().count() << endl;
//    cout << "One iter duration = " << ct.duration().count() / kolvo << endl;
//    cout << "Точность " << manipulator.GetDeltaR().Length() + manipulator.GetDeltaS().Length()<< endl;

    delete[] T_ptr;
    delete[] Ji_ptr;
    delete[] U_ptr;
    delete[] speed_ptr;

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
    StateVector tarv3(-0.35, -0.3341, 0.0515, ToRad(178), ToRad(0), ToRad(0));
    StateVector tarv4(-0.45, -0.3341, 0.0515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv5( 0.35, -0.1341, 0.1515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv6( 0.15, +0.3341, 0.2515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv7(-0.35, +0.1341, 0.1515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv8(-0.35, -0.3341, 0.2515, ToRad(178), ToRad(0), ToRad(0));
//    StateVector tarv(0.3, -0.2341, 0.1515, ToRad(90), ToRad(+90),   ToRad(+90));

//    try
//    {
//        solver.SolveInverseKinematic(10e-5, stv, start_an, start_an);
//        solver.SolveInverseKinematic(10e-5, tarv, start_an, res_an);
//    }
//    catch (std::string &str)
//    {
//        cout << str << endl;
//    }

//    MeshCollider cube;
//    MeshCollider cube2;
//    cube.ReadStl("STLmodels/cube2m.stl");
//    cube2.ReadStl("STLmodels/cube2m.stl");
//    cube2.MoveMesh(Quaternion(0.707, 0, 0, 0.707), Vector3(2, 2, 0));
////    cube2.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(1.9, 1.9, 0));
//    if (cube.CheckCollision(cube2))
//        cout << "ALARM!" << endl;



//    solver.SolveDirectKinematic(angle_stop, TT);
    Polynom result[1000];
    scalar v_tcp(0.18);
    scalar a_tcp(1);
    CheckTime ct1;
    try {
        solver.CalcPolynomLine(stv,   tarv,  v_tcp, a_tcp, angle_start, res_an, 10, result);
        solver.CalcPolynomLine(tarv,  tarv1, v_tcp, a_tcp, res_an,      res_an, 10, result);
        solver.CalcPolynomLine(tarv1, tarv2, v_tcp, a_tcp, res_an,      res_an, 10, result);
        solver.CalcPolynomLine(tarv2, tarv3, v_tcp, a_tcp, res_an,      res_an, 10, result);
        solver.CalcPolynomLine(tarv3, tarv4, v_tcp, a_tcp, res_an,      res_an, 10, result);
    }  catch (std::string &str) {
        cout << str << endl;
    }

    cout << " Time = " << ct1.duration().count() << endl;


//    solver.CalcPolynomLine(tarv2, tarv3, res_an,    res_an, 10, result);
//    solver.CalcPolynomLine(tarv1, tarv2, res_an,    res_an, 3000, 30, result);
//    solver.CalcPolynomLine(tarv2, tarv3, res_an,    res_an, 3000, 30, result);
//    solver.CalcPolynomLine(tarv3, tarv4, res_an,    res_an, 3000, 30, result);
//    solver.CalcPolynomLine(tarv4, tarv5, res_an,    res_an, 3000, 30, result);
//    solver.CalcPolynomLine(tarv5, tarv6, res_an,    res_an, 3000, 30, result);
//    solver.CalcPolynomLine(tarv6, tarv7, res_an,    res_an, 3000, 30, result);
//    solver.CalcPolynomLine(tarv7, tarv8, res_an,    res_an, 3000, 30, result);

//    pol.CalcPolynom(&start_an, &res_an, 3000);
//    Emulator emulator(manipulator);
//    emulator.MoveManip(pol, 3000);

//    manipulator.CalcPolynomLine(&angle_start, &angle_stop, ti, pol);
//    manipulator.CheckTrajectoryForLinear(pol, Vector3(0.67, 0.2431, 0.0615), target, ti, 0.01, 0.01);
//    cout << manipulator.CalcDistancePointToLine(Vector3(22,2,3), Vector3(6, 7, 8), Vector3(11, 12, 13)) << endl;

//#define STL_READER_NO_EXCEPTION
//    stl_reader::StlMesh<float, unsigned int> mesh;
//    try {
//        mesh.read_file("joint2.stl");

//        for (size_t itri = 0; itri < mesh.num_tris(); ++itri)
//        {
//            std::cout << "coordinates of triangle" << itri << " : ";
//            for (size_t icorner = 0; icorner < 3; ++icorner)
//            {
//                const float *c = mesh.vrt_coords(mesh.tri_corner_ind(itri, icorner));
//                std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ")";
//            }
//        std::cout << std::endl;
//        const float* n = mesh.tri_normal(itri);
//        std::cout   << "normal of triangle " << itri << ": "
//                    << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
//        }
//    }  catch (std::exception &e) {
//        std::cout << e.what() << std::endl;
//    }
//    MeshCollider mc_j1, mc_j2, mc_j3, mc_j4, mc_j5, mc_j6;
//    MeshCollider mc_l1, mc_l2, mc_l3, mc_b;
//    mc_j2.ReadStl("STLmodels/joint2.stl");
//    mc_j2.MoveMesh(Quaternion(0, 0, 0.383, 0.924), Vector3(0, 0, 1));
//    mc_j3.read_stl("STLmodels/joint3.stl");
//    mc_j4.read_stl("STLmodels/joint4.stl");
//    mc_j5.read_stl("STLmodels/joint5.stl");
//    mc_j6.read_stl("STLmodels/joint6.stl");
//    mc_l2.read_stl("STLmodels/link2.stl");
//    mc_l3.read_stl("STLmodels/link3.stl");
//    mc_b.read_stl("STLmodels/base.stl");
//    GJ

//    mc_j2.MoveMesh();


#ifdef WITH_TESTS
    QTest::qExec(new test_my_math, argc, argv);
#endif
    cout << "!!!___FINISH___!!!" << endl;
    return app.exec();
//    return 0;
}
#endif
