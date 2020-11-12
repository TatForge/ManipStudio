#include <main.h>
#include <qdebug.h>
#include <array>
#include <chrono>
#include <string.h>

#include "robo.h"

#define RAD M_PI/180
#define DEG 180/M_PI

using namespace RoboMath;

int main(int argc, char *argv[])
{
//    matrix a {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
//    matrix b {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
//    a.matrixCout();
//    matrix c = a * b;
//    c.matrixCout();
//    a.getColsCount();
//    a.getRowsCount();
//    a.elem(1, 1);
//    a.matrixOut();

    Quaternion q(Vector3(1, 0, 0), -60 * RAD);
    Vector3 v(0, 120, 0);
    v = q.RotateVector(v);



    array<scalar, 6> tet {0 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180};
//    Matrix A {{0,   -M_PI/2, 232.5,  tet[0]},
//              {375, 0,       0,      tet[1]},
//              {295, 0,       0,      tet[2]},
//              {0,   -M_PI/2, 120.5,  tet[3]},
//              {0,   M_PI/2,  171,    tet[4]},
//              {0,   0,       122.6,  tet[5]},
//              {0,   0,       100,    tet[6]}};
    Matrix A {{0,   -M_PI/2, 232.5,  tet[0]},
              {375, 0,       0,      tet[1]},
              {295, 0,       0,      tet[2]},
              {0,   -M_PI/2, 120.5,  tet[3]},
              {0,   M_PI/2,  171,    tet[4]},
              {0,   0,       122.6,  tet[5]}};


    scalar *target = new scalar [3] {0.1, 0.1, 0.4};

//    int *Jacobian = new int [18];
    Matrix Jacobian(3, 5);
    scalar dampingLambda = 0.01;
    scalar DLS = dampingLambda * dampingLambda;

    array<Elem, 6> manip ;
    manip[0] = Elem(Vector3(0, 0, 0),               unit_z,  0);
    manip[1] = Elem(Vector3(0,     0,      0.2325), unit_y,  0);
    manip[2] = Elem(Vector3(0.375, 0,      0.2325), unit_y,  0);
    manip[3] = Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  0);
    manip[4] = Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, 0);
    manip[5] = Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  0);
    initializer_list<Elem> il_elem{{Elem(Vector3(0,     0,      0),      unit_z,  0)},
                                   {Elem(Vector3(0,     0,      0.2325), unit_y,  -90 * RAD)},
                                   {Elem(Vector3(0.375, 0,      0.2325), unit_y,  0 * RAD)},
                                   {Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  0 * RAD)},
                                   {Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, 0 * RAD)},
                                   {Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  0)}};
    Robo manipulator(il_elem);
    manipulator.SetConfigurationMatrix(A);
    manipulator.RoboCout();
    manipulator.CalculateJacobian();
    manipulator.CalculateDeltaS(RoboMath::Vector3({0.1, 0.1, 0.4}));
    manipulator.CalculateDeltaThetas();
    manipulator.DeltaThetasCout();
    manipulator.ComputeJointsPosition();
    manipulator.UpdateJointsLocalPosition();

    manipulator.RoboCout();



    double x, y, z, nx, ny, nz;
    for (int i = manip.size() - 1; i > 0; i--)
    {
        //creating JEndMatrix
        x = (manip[i-1].GetPosition().x - manip[5].GetPosition().x);
        y = (manip[i-1].GetPosition().y - manip[5].GetPosition().y);
        z = (manip[i-1].GetPosition().z - manip[5].GetPosition().z);
        nx = y*manip[i - 1].GetPosition().x - z*manip[i - 1].GetPosition().x;//(*(target + 0) - manip[i].getX());
        ny = z*manip[i - 1].GetPosition().y - x*manip[i - 1].GetPosition().y;//(*(target + 1) - manip[i].getY());
        nz = x*manip[i - 1].GetPosition().z - y*manip[i - 1].GetPosition().z;//(*(target + 2) - manip[i].getZ());
        Jacobian(0, i - 1) = nx;//y*manip[i - 1].getOrZ() - z*manip[i - 1].getOrY();//(*(target + 0) - manip[i].getX());
        Jacobian(1, i - 1) = ny;//z*manip[i - 1].getOrX() - x*manip[i - 1].getOrZ();//(*(target + 1) - manip[i].getY());
        Jacobian(2, i - 1) = nz;//x*manip[i - 1].getOrY() - y*manip[i - 1].getOrX();//(*(target + 2) - manip[i].getZ());
        cout << "Jacobian " << i << " col "
             << Jacobian(0, i - 1) << "\t"
             << Jacobian(1, i - 1) << "\t"
             << Jacobian(2, i - 1) << endl;
    }

    //matrixMultiplyTranspose
    //doing transpose J matrix
    Matrix JacobianTransposed = Jacobian;
    JacobianTransposed = Jacobian;
    JacobianTransposed.Transpose();
    Matrix U = Jacobian * JacobianTransposed;
    U.AddDiagonal(DLS);
    //Solving U matrix
    Matrix AM(3, 4);
    Vector dS{*(target + 0) - manip[5].GetPosition().x, *(target + 1) - manip[5].GetPosition().y, *(target + 2) - manip[5].GetPosition().z};
    AM.SetSubmatrix(U);
    AM.SetColumn(dS, 4);
    AM.SolveGauss();
    Vector dT(3);
    AM.GetTheRoots(dT);
    for (int i = 0; i < dT.GetLength(); i++)
    {
        cout << "dT " << *(dT.GetDataPtr() + i) << endl;
    }
    Vector dTet(5);
    dTet = JacobianTransposed * dT;


{
//    array<matrix, 6> T1;
//    T1[0] = {{cos(A.elem(0, 3)), - sin(A.elem(0, 3)) * cos(A.elem(0, 1)),   sin(A.elem(0, 3)) * sin(A.elem(0, 1)), A.elem(0, 0) * cos(A.elem(0, 3))},
//             {sin(A.elem(0, 3)),   cos(A.elem(0, 3)) * cos(A.elem(0, 1)), - cos(A.elem(0, 3)) * sin(A.elem(0, 1)), A.elem(0, 0) * sin(A.elem(0, 3))},
//             {0,                   sin(A.elem(0, 1)),                       cos(A.elem(0, 1)),                     A.elem(0, 2)},
//             {0,                   0,                                       0,                                     0}};
//    T1[0].matrixCout();
//    cout << " T0 _______________" << endl;
//    T1[1] = {{cos(A.elem(1, 3)), - sin(A.elem(1, 3)) * cos(A.elem(1, 1)),   sin(A.elem(1, 3)) * sin(A.elem(1, 1)), A.elem(1, 0) * cos(A.elem(1, 3))},
//             {sin(A.elem(1, 3)),   cos(A.elem(1, 3)) * cos(A.elem(1, 1)), - cos(A.elem(1, 3)) * sin(A.elem(1, 1)), A.elem(1, 0) * sin(A.elem(1, 3))},
//             {0,                   sin(A.elem(1, 1)),                       cos(A.elem(1, 1)),                     A.elem(1, 2)},
//             {0,                   0,                                       0,                                     0}};
//    T1[0].matrixCout();
//    T1[1].matrixCout();
//    cout << " T1 _______________" << T1.size() << endl;
//    T1[2] = {{cos(A.elem(2, 3)), - sin(A.elem(2, 3)) * cos(A.elem(2, 1)),   sin(A.elem(2, 3)) * sin(A.elem(2, 1)), A.elem(2, 0) * cos(A.elem(2, 3))},
//             {sin(A.elem(2, 3)),   cos(A.elem(2, 3)) * cos(A.elem(2, 1)), - cos(A.elem(2, 3)) * sin(A.elem(2, 1)), A.elem(2, 0) * sin(A.elem(2, 3))},
//             {0,                   sin(A.elem(2, 1)),                       cos(A.elem(2, 1)),                     A.elem(2, 2)},
//             {0,                   0,                                       0,                                     0}};
//    T1[0].matrixCout();
//    T1[1].matrixCout();
//    T1[2].matrixCout();
//    cout << " T2 _______________" << T1.size() << endl;
}
    array<Matrix, 6> T;

    auto start = std::chrono::high_resolution_clock::now();
    for (counter i = 0; i < T.size(); i++)
    {
        T[i] = {{cos(A.elem(i, 3)), -sin(A.elem(i, 3)) * cos(A.elem(i, 1)),   sin(A.elem(i, 3)) * sin(A.elem(i, 1)), A.elem(i, 0) * cos(A.elem(i, 3))},
                {sin(A.elem(i, 3)),  cos(A.elem(i, 3)) * cos(A.elem(i, 1)), - cos(A.elem(i, 3)) * sin(A.elem(i, 1)), A.elem(i, 0) * sin(A.elem(i, 3))},
                {0,                  sin(A.elem(i, 1)),                       cos(A.elem(i, 1)),                     A.elem(i, 2)},
                {0,                  0,                                       0,                                     1}};

    }
    /*
    for (size_t i = 0; i < T.size(); i++)
    {
        T[i].matrixCout();
        cout << "T[" << i <<"]_________________" << endl;
    }*/
    array<Matrix, 6> T6;
    T6[0] = T[0];
    for (size_t i = 1; i < T6.size(); i++)
    {
        T6[i] = T6[i-1] * T[i];
    }
//    matrix T6[0] = T[0];
//    matrix T2 = T1 * T[1];
//    matrix T3 = T2 * T[2];
//    matrix T4 = T3 * T[3];
//    matrix T5 = T4 * T[4];
//    matrix T6 = T5 * T[5];
    Matrix L = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
//    L.matrixCout();
    cout << " X = " << L.elem(0, 3) << " Y = " << L.elem(1, 3) << " Z = "<< L.elem(2, 3) << endl;
    cout << " Tet =  " << DEG * acos(L.elem(2,2)) <<
            " Fi  =  " << DEG * acos(- L.elem(1,2) / sin(L.elem(2, 2))) <<
            " Psi =  " << DEG * acos(L.elem(2,1) / sin(L.elem(2, 2))) << endl;





//    **************************************************************
//    *                                                            *
//    *                                                            *
//    **************************************************************

//    target = {700, 400, 100};

//    vector<double> JacEnd(18);
//    vector<double> JacTarget(18);
//    cout << endl;
//    for (int i = 5; i >= 0 ; i--)
//    {
//        cout << "Iter count № " << i
//             << " T6.x = " << T6[i].elem(0, 3)
//             << " T6.y = " << T6[i].elem(1, 3)
//             << " T6.z = " << T6[i].elem(2, 3)
//             << endl;
//        JacTarget[i * 3] = T6[i].elem(0, 3) - target[0];
//        JacTarget[i * 3 + 1] = T6[i].elem(1, 3) - target[1];
//        JacTarget[i * 3 + 2] = T6[i].elem(2, 3) - target[2];
//        cout << " JacT.x = " << JacTarget[i * 3]
//             << " JacT.y = " << JacTarget[i * 3 + 1]
//             << " JacT.z = " << JacTarget[i * 3 + 2]
//             << endl;
//    }

//    double kkk = 0;

    return 0;
}

