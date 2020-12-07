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
    array<scalar, 6> tet {0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180,
                          0 * M_PI/180};
    Matrix A {{0,   -M_PI/2,  232.5,  tet[0]},
              {0,    0,       0,      tet[1]},
              {375,  0,       0,      tet[2]},
              {295, -M_PI/2,  120.5,  tet[3]},
              {0,    M_PI/2,  171,    tet[4]},
              {0,    0,       122.6,  tet[5]}};

    array<Matrix, 6> T;
    for (counter i = 0; i < T.size(); i++)
    {
        T[i] = {{cos(A.at(i, 3)), -sin(A.at(i, 3)) * cos(A.at(i, 1)),   sin(A.at(i, 3)) * sin(A.at(i, 1)), A.at(i, 0) * cos(A.at(i, 3))},
                {sin(A.at(i, 3)),  cos(A.at(i, 3)) * cos(A.at(i, 1)), - cos(A.at(i, 3)) * sin(A.at(i, 1)), A.at(i, 0) * sin(A.at(i, 3))},
                {0,                  sin(A.at(i, 1)),                       cos(A.at(i, 1)),                     A.at(i, 2)},
                {0,                  0,                                       0,                                     1}};

    }
    Matrix L = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];


    //Dik for 3 elem i = 3, k = 3
    const Matrix Q = {{0, -1, 0, 0},
                      {1,  0, 0, 0},
                      {0,  0, 0, 0},
                      {0,  0, 0, 0}};
    Matrix test = {{1,2,3,4},{5,6,7,8},{9,10,11,12},{13,14,15,16}};

            //Ixx  Iyy  Izz  Ixy   Ixz
    Tensor I {0.1, 0.2, 0.3, 0.25, 0.14, 0.28};
    //Массы элементов
    Vector m {1, 2, 3, 4, 5, 6};
    //Центры масс каждого элемента в локальной СК
    array<Vector, 6> cm;
    Vector gv{0, 0, -g, 0};
    for (int i = 0; i < 6; i++)
    {
        cm[i] = Vector({1.22 + i * 0.1, 1.34 + i * 0.04, 1.65 + i * 0.12, 1});
    }

    int i = 4;//4тый джоинт
    int n = 5;
    Matrix Ji = {{(-I.xx() + I.yy() + I.zz())/2, I.xy(), I.xz(), m(i) * cm[i](0)},
                 {I.xy(), (I.xx() - I.yy() + I.zz())/2,  I.yz(), m(i) * cm[i](1)},
                 {I.xz(), I.yz(), (I.xx() + I.yy() - I.zz())/2,  m(i) * cm[i](2)},
                 {m(i) * cm[i](0),  m(i) * cm[i](1), m(i) * cm[i](2), m(i)}};

    Ji.MatrixCout();
    scalar Dik(0);
    array<Matrix, 6> dAj;/*
    Matrix Ujk(4, 4);
    Ujk.MakeIdentity();*/
    Matrix Uji(4, 4);
    Uji.MakeIdentity();
    Matrix D(4,4);
    D.MakeZero();
    L.MatrixCout();
    Matrix KK = Q * L;
    KK.MatrixCout();
    //для i = 4, k = 1

//    Ujk = T[0] * T[1] * Q * T[2] * T[3];
//    Ujk.MatrixCout();

    MatrixOfMatrix6x6 U ;
//    //--расчет U
//    for (int i = 0; i < 6; i++)
//    {
//        for (int j = 0; j < 6; j++)
//        {
//            for (int k = 0; k <= j; k++)
//            {
//                if (k == j)
//                    U(i, j) = U(j, k) * Q * T[ctr - 1];
//            }
//        }
//    }


    for (int k = 1; k <= 6; k++)
    {
        for (int j = k; j <= 6; j++)
        {
            //Считается Ujk для конкретного i,j,k
            for (counter ctr = 0; ctr < j; ctr++)
            {
                if (ctr == k - 1)
                    U(j - 1, k - 1) = U(j - 1, k - 1) * Q * T[ctr];
                else
                {
                    U(j - 1, k - 1) = U(j - 1, k - 1) * T[ctr];
                }
//                U(j - 1, k - 1).MatrixCout();
            }
//            cout << "U" << j << k << endl;
//            U(j - 1, k - 1).MatrixCout();
        }
    }
    //__расчет U

//    for (int i = 0; i < 6; i++)
//        T[i].MatrixCout();
//    for (int k = 3; k < 6; k++)
//    {
//        for (int j = max(i, k); j < 6; j++)
//        {
//            //Считается Ujk для конкретного i,j,k
//            for (counter ctr = 1; ctr <= j; ctr++)
//            {
//                if (ctr == k - 1)
//                    U(j, k) = U(j, k) * Q * T[ctr - 1];
//                else
//                    U(j,k) = U(j,k) * T[ctr - 1];
//                U(j,k).MatrixCout();
//            }
//            //Считается Uji для конкретного i,j,k
//            int *isi = new int;
//            for (counter ctr = 1; ctr <= j; ctr++)
//            {
//                if (ctr == i - 1)
//                    Uji = Uji * Q * T[ctr - 1];
//                        else
//                    Uji = Uji * T[ctr - 1];
//                Uji.MatrixCout();
//            }
//            Uji.Transpose();
//            U(j,k).MatrixCout();
//            Ji.MatrixCout();
//            Uji.MatrixCout();
//            auto temp = U(j,k) * Ji * Uji;
//            temp.MatrixCout();
//            Dik += Trace(U(j,k) * Ji * Uji);
//        }
//    }
    Vector Dv(6, 0);
    //Расчет D для i-го элемента манипулятора
    for (int i = 0; i < 6; i++)
        for (int k = i; i < 6; i++)
        {
            for (int j = max(i, k); j < 6; j++)
            {
                Dv(i) += Trace(U(j,k) * Ji * U(j,i));
            }
        }
    Dv.VectorCout("D vector");

    Matrix m3 {{1,2,3},{5,6,7},{7,8,9},{4,3,2},{7,6,5}};
//    Matrix n3 {{3,4,5,5,4},{7,6,5,4,1},{9,3,2,6,3}};
    m3.MultiplyTranspose(m3).MatrixCout();

//    Matrix ee{{1,2,3},{4,5,6},{7,8,9},{10,11,12},{13,14,15}};
//    ee.MatrixCout();
//    Matrix uu{{1,2,3},{4,5,6},{7,8,9},{10,11,12},{13,14,15}, {11,12,13}};
//    Vector vv{3,4,5};
//    uu.MatrixCout();
//    auto iii = ee.MultiplyTranspose(uu);
//    auto oo = ee.MultiplyTranspose(vv);
//    oo.VectorCout("vect");
//    oo = U(1,1).MultiplyTranspose(cm[0]);
//    U(1,1).MatrixCout();
//    oo.VectorCout("vect");


    Vector p(6,0);
    int ctr(0);
    for (int i = 0; i < 6; i++)
    {
        for (int j = ctr; j < 6; j++)
        {
//            U(j,i).MatrixCout();
//            cm[j].VectorCout("cm");
//            gv.VectorCout("gv");

            p(i) += - m(j) * gv.MultiplyTranspose(U(j,i).MultiplyTranspose(cm[j]));
        }
        ctr++;
    }
    p.VectorCout("P");
    Vector E(6,0);
    {
    auto asd = m(0) * gv.MultiplyTranspose(U(0,0).MultiplyTranspose(cm[0]));
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
        cout << "Вектор p считается правильно" << endl;

    i = 3;
    Matrix hikm(4,4);
    Matrix u_buff(4,4);
    u_buff.MakeIdentity();
    for (int j = i; j < n; j++)
    {
        for (int k = 1; k < j; k++)
        {
            for (int p = 1; p < j; p++)
            {
                if (k >= p)
                {

                }
                else
                {

                }
            }
        }
    }


    for (int k = 1; k <= 6; k++)
    {
        for (int j = k; j <= 6; j++)
        {
            //Считается Ujk для конкретного i,j,k
            for (counter ctr = 0; ctr < j; ctr++)
            {
                if (ctr == k - 1)
                    U(j - 1, k - 1) = U(j - 1, k - 1) * Q * T[ctr];
                else
                {
                    U(j - 1, k - 1) = U(j - 1, k - 1) * T[ctr];
                }
//                U(j - 1, k - 1).MatrixCout();
            }
//            cout << "U" << j << k << endl;
//            U(j - 1, k - 1).MatrixCout();
        }
    }


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

    QTest::qExec(new test_my_math, argc, argv);
    return 0;
}

