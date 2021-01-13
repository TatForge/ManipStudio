#include "test_my_math.h"
#include "robo.h"
#include "cuda_math.h"

#include <main.h>
#include <array>
#include <chrono>
#include <string.h>
#include <thread>

#ifdef WITH_TESTS
#include <QtTest/QTest>
#endif


//#include "commandworker.h"


using namespace RoboMath;

int main(int argc, char *argv[])
{
    array<scalar, 6> tet {0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180};
    Vector speed{1,1,1,1,1,1};
    Vector acceleration{1,1,1,1,1,1};
//    Vector speed{0,1,0,0,0,0};
//    Vector acceleration{0,0,0,0,0,0};

    Matrix A {{0,     -M_PI/2, 0.2325, tet[0]},
              {0.375,  0,      0.100,  tet[1]},
              {0.295,  0,     -0.100,  tet[2]},
              {0,     -M_PI/2, 0.1205, tet[3]},
              {0,      M_PI/2, 0.171,  tet[4]},
              {0,      0,      0.1226, tet[5]}};

    array<Matrix, 6> T;
    {
    T[0] = {{cos(A(0,3)),         -sin(A(0,3)),           0,       A(0,0)},
            {sin(A(0,3)) * cos(0), cos(A(0,3)) * cos(0), -sin(0), -sin(0) * A(0,2)},
            {sin(A(0,3)) * sin(0), cos(A(0,3)) * sin(0),  cos(0),  cos(0) * A(0,2)},
            {0,                    0,                     0,       1}};
//    T[0].MatrixCout();
    for (counter i = 1; i < T.size(); i++)
    {
        T[i] = {{cos(A(i,3)),                 -sin(A(i,3)),                0,              A(i-1,0)},
                {sin(A(i,3)) * cos(A(i-1,1)), cos(A(i,3)) * cos(A(i-1,1)), -sin(A(i-1,1)), -sin(A(i-1,1)) * A(i,2)},
                {sin(A(i,3)) * sin(A(i-1,1)), cos(A(i,3)) * sin(A(i-1,1)), cos(A(i-1,1)),  cos(A(i-1,1)) * A(i,2)},
                {0,                           0,                           0,              1}};
//        T[i].MatrixCout("T");
    }
    }
    Matrix L =  T[0] * T[1] * T[2] * T[3] * T[4]* T[5];

    const Matrix Q = {{0, -1, 0, 0},
                      {1,  0, 0, 0},
                      {0,  0, 0, 0},
                      {0,  0, 0, 0}};
    Matrix test = {{1,2,3,4},{5,6,7,8},{9,10,11,12},{13,14,15,16}};

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
    //Центры масс каждого элемента в локальной СК

    Vector gv{0, 0, -g, 0};



    array<Matrix, 6> dAj;

    Matrix Uji(4, 4);
    Uji.MakeIdentity();

    MatrixOfMatrix6x6 U;

    Vector res_hikm(6,0);
    scalar hikm_buff;
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


    initializer_list<Elem> il_elem {Elem(Vector3(0,     0,      0),      unit_z,  ToRad(0)),
                                    Elem(Vector3(0,     0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.375, 0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, ToRad(0)),
                                    Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  ToRad(0))};
    Robo manipulator(il_elem);
    manipulator.ComputeJointsPosition();
    manipulator.CalcMoments();
//    cout << "Duration " << tim.duration().count() << endl;
//    hikm.VectorCout("hikm");
//    Dv.VectorCout("Dv");
//    p.VectorCout("p");

    checkTime ct;
    int kolvo = 0;
    for (int i = 0; i < kolvo; i ++)
    {
        bool solved = true;
        Vector3 target(sin(i) * 0.3, cos(i) * 0.2, cos(i) * 0.1);
//        Vector3 target(0.67, 0.2431, 0.0615);
//        Vector3 target_r(ToRad(90), ToRad(30), ToRad(0));
        Vector3 target_r(sin(i) * ToRad(90), cos(i) * ToRad(80), cos(i) * ToRad(-30));
//        cout << " x = " << manipulator.GetEffector()->GetPosition().x
//             << " y = " << manipulator.GetEffector()->GetPosition().y
//             << " z = " << manipulator.GetEffector()->GetPosition().z
//             << endl;
        manipulator.CalculateDeltaR(target_r);
        manipulator.CalculateDeltaS(target);
        counter ctr(0);
        while(manipulator.GetDeltaR().Length() + manipulator.GetDeltaS().Length() > 10e-5)
        {
            manipulator.CalculateJacobian();
            manipulator.CalculateDeltaThetas();
            manipulator.AddDeltaThetas();
            manipulator.ComputeJointsPosition();
            manipulator.CalculateDeltaR(target_r);
            manipulator.CalculateDeltaS(target);
            ctr++;
            if (ctr > 100)
            {
                cout << "Уравнение не сошлось !_!_!_!_!_!_!_!_!_!_!_!_!" << endl;
                cout << "Цель не достижима _______________" << endl;
                cout << "Положение эффектора " << endl
                     << " ang1 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().x)
                     << " ang2 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().y)
                     << " ang3 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().z)
                     << " x = " << manipulator.GetEffectorPosition().x
                     << " y = " << manipulator.GetEffectorPosition().y
                     << " z = " << manipulator.GetEffectorPosition().z
                     << endl ;
                cout << " Положение цели \n"
                     << " ang1 = " << ToDeg(target_r.x)
                     << " ang2 = " << ToDeg(target_r.y)
                     << " ang3 = " << ToDeg(target_r.z)
                     << " x = " << target.x
                     << " y = " << target.y
                     << " z = " << target.z
                     << endl;
                solved = false;
                break;
            }
        }

//        auto d = ct.duration().count();
        if (solved)
        {
//            cout << "Уравнение сошлось за " << ctr << " решений" << endl;
//            cout << "Положение эффектора " << endl
//                 << " ang1 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().x)
//                 << " ang2 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().y)
//                 << " ang3 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().z)
//                 << " x = " << manipulator.GetEffectorPosition().x
//                 << " y = " << manipulator.GetEffectorPosition().y
//                 << " z = " << manipulator.GetEffectorPosition().z
//                 << endl ;
//            cout << "Положение цели \n"
//                 << " ang1 = " << ToDeg(target_r.x)
//                 << " ang2 = " << ToDeg(target_r.y)
//                 << " ang3 = " << ToDeg(target_r.z)
//                 << " x = " << target.x
//                 << " y = " << target.y
//                 << " z = " << target.z
//                 << endl;
        }
//        cout << "Время расчета составило " << d << " секунд" << endl << endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

    cout << "Time of " << kolvo << " calculations " << ct.duration().count() << endl;
    cout << "One iter duration = " << ct.duration().count() / kolvo << endl;
    cout << "Точность " << manipulator.GetDeltaR().Length() << endl;
#ifdef WITH_TESTS
    QTest::qExec(new test_my_math, argc, argv);
#endif
    return 0;
}

