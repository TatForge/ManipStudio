#include <main.h>
#include <array>
#include <chrono>
#include <string.h>
#include <thread>
#include <QTest>

#include "test_my_math.h"
#include "robo.h"
//#include "commandworker.h"


using namespace RoboMath;

int main(int argc, char *argv[])
{


//    temp_1 = *new Matrix(4,4);
//    temp_2 = *new Matrix(4,4);
//    temp_1.MakeIdentity();
//    temp_2.MakeIdentity();




    Matrix lolol{{1,1,1,1},
                 {1,1,1,1},
                 {1,1,1,1},
                 {1,1,1,1}};
    Matrix memem{{2,2,2,2},
                 {2,2,2,2},
                 {2,2,2,2},
                 {2,2,2,2}};
    Matrix opopo{{3,3,3,3},
                 {3,3,3,3},
                 {3,3,3,3},
                 {3,3,3,3}};
    Matrix rrrr(4,4);
    checkTime chktmr;
    for (int i = 0; i < 1000; i++)
    {
        for(int j = 0; j < 1000; j++)
        {
            rrrr =  lolol * memem * opopo * lolol * lolol * memem * opopo * lolol * lolol * memem * opopo * lolol;
        }
    }
    cout << "Duration 1000000 matrix multiplication = " << chktmr.duration().count() << " sek" << endl;




    double kekeke = 123;










    array<scalar, 6> tet {0 * M_PI/180,
                          -90 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180};
//    Vector speed{1,1,1,1,1,1};
//    Vector acceleration{1,1,1,1,1,1};
    Vector speed{0,0,0,0,0,0};
    Vector acceleration{0,0,0,0,0,0};

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
    //Старая версия
//    for (counter i = 0; i < T.size(); i++)
//    {
//        T[i] = {{cos(A.at(i, 3)), -sin(A.at(i, 3)) * cos(A.at(i, 1)),   sin(A.at(i, 3)) * sin(A.at(i, 1)), A.at(i, 0) * cos(A.at(i, 3))},
//                {sin(A.at(i, 3)),  cos(A.at(i, 3)) * cos(A.at(i, 1)), - cos(A.at(i, 3)) * sin(A.at(i, 1)), A.at(i, 0) * sin(A.at(i, 3))},
//                {0,                  sin(A.at(i, 1)),                       cos(A.at(i, 1)),                     A.at(i, 2)},
//                {0,                  0,                                       0,                                     1}};
//        T[i].MatrixCout("Ti");

//    }

    Matrix L =  T[0] * T[1] * T[2] * T[3] * T[4]* T[5];
//    L.MatrixCout();
    //Dik for 3 elem i = 3, k = 3
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
    array<Vector, 6> cm;
    Vector gv{0, 0, -g, 0};

//    cm[0] = Vector({0, 0, 0.115, 1});
//    cm[1] = Vector({0.158, 0, 0, 1});
//    cm[2] = Vector({0.137, 0, 0, 1});
//    cm[3] = Vector({0, 0, 0.06 , 1});
//    cm[4] = Vector({0, 0, 0.085 , 1});
//    cm[5] = Vector({0, 0, 0.61 , 1});
    cm[0] = Vector({0, 0, 0, 1});
    cm[1] = Vector({0, 0, 0, 1});
    cm[2] = Vector({0, 0, 0, 1});
    cm[3] = Vector({0, 0, 0, 1});
    cm[4] = Vector({0, 0, +0.085, 1});
    cm[5] = Vector({0, 0, 0, 1});

    int i = 4;//4тый джоинт
    int n = 5;
    scalar Dik(0);
    array<Matrix, 6> dAj;/*
    Matrix Ujk(4, 4);
    Ujk.MakeIdentity();*/
    Matrix Uji(4, 4);
    Uji.MakeIdentity();
    Matrix D(4,4);
    D.MakeZero();
    L.MatrixCout("L Matrix");
    Matrix KK = Q * L;
//    KK.MatrixCout();
    //для i = 4, k = 1

    MatrixOfMatrix6x6 U ;

    Vector res_hikm(6,0);
    scalar hikm_buff;
    array<Matrix, 6> Ji;
    Vector Dv(6, 0);
    Vector p(6,0);
    Vector hikm(6,0);
    Matrix u_buff(4,4);
    Vector tao(6,0);

    checkTime tim;
//    for (counter k = 1; k <= 6; k++)
//    {
//        for (counter j = k; j <= 6; j++)
//        {
//            //Считается Ujk для конкретного i,j,k
//            for (counter ctr = 0; ctr < j; ctr++)
//            {
//                if (ctr == k - 1)
//                    U(j - 1, k - 1) = U(j - 1, k - 1) * Q * T[ctr];
//                else
//                {
//                    U(j - 1, k - 1) = U(j - 1, k - 1) * T[ctr];
//                }
//                U(j - 1, k - 1).MatrixCout();
//            }
//            cout << "U" << j << k << endl;
//            U(j - 1, k - 1).MatrixCout();
//        }
//    }
        for (counter k = 0; k < 6; k++)
        {
            for (counter j = k; j < 6; j++)
            {
                //Считается Ujk для конкретного i,j,k
                for (int ctr = 0; ctr <= j; ctr++)
                {
                    if (ctr == k)
                        U(j, k) = U(j, k) * T[ctr] * Q;
                    else
                    {
                        U(j, k) = U(j, k) * T[ctr];
                    }
//                    U(j, k).MatrixCout();
                }
//                cout << "U" << j << k << endl;
//                U(j, k).MatrixCout();
            }
        }
    //__расчет U

    for (int i = 0; i < 6; i++)
    {
        Ji[i] = {{(-I[i].xx() + I[i].yy() + I[i].zz())/2, I[i].xy(), I[i].xz(), m(i) * cm[i](0)},
                 {I[i].xy(), (I[i].xx() - I[i].yy() + I[i].zz())/2,  I[i].yz(), m(i) * cm[i](1)},
                 {I[i].xz(), I[i].yz(), (I[i].xx() + I[i].yy() - I[i].zz())/2,  m(i) * cm[i](2)},
                 {m(i) * cm[i](0),  m(i) * cm[i](1), m(i) * cm[i](2), m(i)}};
//        Ji[i].MatrixCout();
    }


    //Расчет D для i-го элемента манипулятора
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = i; i < 6; i++)
        {
            for (counter j = max(i, k); j < 6; j++)
            {
                Dv(i) += Trace(U(j,k) * Ji[k].MultiplyTranspose(U(j,i)));
            }
            Dv(i) *= acceleration(k);
        }
    }
//    Dv.VectorCout("D vector");

    int ctr(0);
    for (counter i = 0; i < 6; i++)
    {
        for (counter j = i; j < 6; j++)
        {
//            cout << i << j<< "__________________________________________________" << endl;
////            U(j,i).MatrixCout();
//            cm[j].VectorCout("cm");

            p(i) += - m(j) * gv.MultiplyTranspose(U(j, i).MultiplyTranspose(cm[j]));
//            cout << " U " << j + 1 << i + 1 << endl;
//            U(j, i).MatrixCout("U matrix");
//            U(j, i).MultiplyTranspose(cm[j]).VectorCout("U * cm");
//            cout << "p = " << p(i) << endl;
        }
        ctr++;
    }
//    p.VectorCout("P");
//    L.MatrixCout();
   /* Vector E(6,0);
    {
    E(0) = - (m(0) * gv.MultiplyTranspose(U(0,0).MultiplyTranspose(cm[0])) +
              m(1) * gv.MultiplyTranspose(U(1,0).MultiplyTranspose(cm[1])) +
              m(2) * gv.MultiplyTranspose(U(2,0).MultiplyTranspose(cm[2])) +
              m(3) * gv.MultiplyTranspose(U(3,0).MultiplyTranspose(cm[3])) +
              m(4) * gv.MultiplyTranspose(U(4,0).MultiplyTranspose(cm[4])) +
              m(5) * gv.MultiplyTranspose(U(5,0).MultiplyTranspose(cm[5])));
    E(1) = - (m(1) * gv.MultiplyTranspose(U(1,1).MultiplyTranspose(cm[1])) +
              m(2) * gv.MultiplyTranspose(U(2,1).MultiplyTranspose(cm[2])) +
              m(3) * gv.MultiplyTranspose(U(3,1).MultiplyTranspose(cm[3])) +
              m(4) * gv.MultiplyTranspose(U(4,1).MultiplyTranspose(cm[4])) +
              m(5) * gv.MultiplyTranspose(U(5,1).MultiplyTranspose(cm[5])));
    E(2) = - (m(2) * gv.MultiplyTranspose(U(2,2).MultiplyTranspose(cm[2])) +
              m(3) * gv.MultiplyTranspose(U(3,2).MultiplyTranspose(cm[3])) +
              m(4) * gv.MultiplyTranspose(U(4,2).MultiplyTranspose(cm[4])) +
              m(5) * gv.MultiplyTranspose(U(5,2).MultiplyTranspose(cm[5])));
    E(3) = - (m(3) * gv.MultiplyTranspose(U(3,3).MultiplyTranspose(cm[3])) +
              m(4) * gv.MultiplyTranspose(U(4,3).MultiplyTranspose(cm[4])) +
              m(5) * gv.MultiplyTranspose(U(5,3).MultiplyTranspose(cm[5])));
    E(4) = - (m(4) * gv.MultiplyTranspose(U(4,4).MultiplyTranspose(cm[4])) +
              m(5) * gv.MultiplyTranspose(U(5,4).MultiplyTranspose(cm[5])));
    E(5) = - (m(5) * gv.MultiplyTranspose(U(5,5).MultiplyTranspose(cm[5])));
    }
    E.VectorCout("E vector");
    if (E == p)
        cout << "Вектор p считается правильно" << endl;*/


//    u_buff.MakeIdentity();
//    for (counter i = 0; i < 6; i++)
//    {
//        for (counter j = i; j < n; j++)
//        {
//            for (counter k = 1; k <= j; k++)
//            {
//                for (counter m = 1; m <= j; m++)
//                {
//                    for (ctr = 0; ctr < j; ctr++)
//                    {
//                        if (ctr == (k - 1) && ctr == (m - 1))
//                            u_buff = u_buff * Q * Q * T[ctr];
//                        else if (ctr == (k - 1) || ctr == (m - 1))
//                            u_buff = u_buff * Q * T[ctr];
//                        else
//                            u_buff = u_buff * T[ctr];
//                    }
//                    hikm(i) += Trace(u_buff * Ji[i].MultiplyTranspose(U(j,i)));// * speed(k - 1) * speed(m- 1);
//                }
//            }
//            u_buff.MakeIdentity();
//        }
//    }


    //НЕ УДАЛЯТЬ, ПОКА НЕ ПРОВЕРЮ ПРАВИЛЬНОСТЬ________________________________________________________
//    Matrix temp(4,4);
//    Matrix temp2(4,4);

//    u_buff.Multiply1(T[ctr], temp2);
//    u_buff.Multiply2(Q, T[ctr], temp, temp2);
//    u_buff.Multiply3(Q, T[ctr], temp, temp2);

    for (counter i = 0; i < 6; i++)
    {
        for (counter k = 0; k < 6; k++)
        {
            for (counter m = 0; m < 6; m++)
            {
                for (counter j = i; j < 6; j++)
                {
                    hikm_buff = 0;
                    u_buff.MakeIdentity();
                    for (counter ctr = 0; ctr < j; ctr++)
                    {
                        if (ctr == (k - 1) && ctr == (m - 1))
                            u_buff = u_buff * Q * Q * T[ctr];
                        else if (ctr == (k - 1) || ctr == (m - 1))
                            u_buff = u_buff * Q * T[ctr];
                        else
                            u_buff = u_buff * T[ctr];
                    }
//                    cout << "u buff ikmj " << i << k << m  << j << endl;
//                    u_buff.MatrixCout("u_buff");
                    hikm_buff += Trace(u_buff * Ji[i].MultiplyTranspose(U(j,i))) * speed(k) * speed(m);
                }
                hikm(i) += hikm_buff;
            }
        }
    }
    //_______________________________________________________________________________________________
//    for (int i = 0; i < 6; i++)
//    {
//        for (int p = 0; p < i; p++)
//        {
//            for (int r = 0; r < i; r++)
//            {
//                hikm(i) += speed(p) * speed(r) * Trace(U(i,p) * Ji[i].MultiplyTranspose(U(i,r)));
//            }
//        }
//        hikm(i) = 0.5 * hikm(i);
//    }
    cout << "Duration " << tim.duration().count() << endl;
    hikm.VectorCout("hikm");
    Dv.VectorCout("Dv");
    p.VectorCout("p");











    for (counter i = 0; i < 6; i++)
    {
        tao(i) = Dv(i) + hikm(i) + p(i);
    }
    tao.VectorCout("TAO");


    initializer_list<Elem> il_elem {Elem(Vector3(0,     0,      0),      unit_z,  ToRad(0)),
                                    Elem(Vector3(0,     0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.375, 0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, ToRad(0)),
                                    Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  ToRad(0))};
    Robo manipulator(il_elem);
    manipulator.ComputeJointsPosition();
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

//    QTest::qExec(new test_my_math, argc, argv);
    return 0;
}

