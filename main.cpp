#include <my_math.h>
#include <qdebug.h>
#include <array>
#include <chrono>

#define TO_RAD M_PI/180
#define TO_DEG 180/M_PI

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
    array<scalar, 6> tet {-90 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180,
                         -90 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180};
    matrix A {{0,   -M_PI/2, 232.5,  tet[0]},
              {375, 0,       0,      tet[1]},
              {295, 0,       0,      tet[2]},
              {0,   -M_PI/2, 120.5,  tet[3]},
              {0,   M_PI/2,  171,    tet[4]},
              {0,   0,       122.6,  tet[5]}};
    A.matrixCout();
    cout << "A matrix ____________________" << endl;

    array <matrix, 4> AA;
    AA[0] = {1,2,3,4,5,6,7,8,9};
    AA[0] = {1,2,3,4,5,6,7,8,9};
    AA[0] = {1,2,3,4,5,6,7,8,9};

    array<matrix, 6> T1;
    T1[0] = {{cos(A.elem(0, 3)), - sin(A.elem(0, 3)) * cos(A.elem(0, 1)),   sin(A.elem(0, 3)) * sin(A.elem(0, 1)), A.elem(0, 0) * cos(A.elem(0, 3))},
             {sin(A.elem(0, 3)),   cos(A.elem(0, 3)) * cos(A.elem(0, 1)), - cos(A.elem(0, 3)) * sin(A.elem(0, 1)), A.elem(0, 0) * sin(A.elem(0, 3))},
             {0,                   sin(A.elem(0, 1)),                       cos(A.elem(0, 1)),                     A.elem(0, 2)},
             {0,                   0,                                       0,                                     0}};
    T1[0].matrixCout();
    cout << " T0 _______________" << endl;
    T1[1] = {{cos(A.elem(1, 3)), - sin(A.elem(1, 3)) * cos(A.elem(1, 1)),   sin(A.elem(1, 3)) * sin(A.elem(1, 1)), A.elem(1, 0) * cos(A.elem(1, 3))},
             {sin(A.elem(1, 3)),   cos(A.elem(1, 3)) * cos(A.elem(1, 1)), - cos(A.elem(1, 3)) * sin(A.elem(1, 1)), A.elem(1, 0) * sin(A.elem(1, 3))},
             {0,                   sin(A.elem(1, 1)),                       cos(A.elem(1, 1)),                     A.elem(1, 2)},
             {0,                   0,                                       0,                                     0}};
    T1[0].matrixCout();
    T1[1].matrixCout();
    cout << " T1 _______________" << T1.size() << endl;
    T1[2] = {{cos(A.elem(2, 3)), - sin(A.elem(2, 3)) * cos(A.elem(2, 1)),   sin(A.elem(2, 3)) * sin(A.elem(2, 1)), A.elem(2, 0) * cos(A.elem(2, 3))},
             {sin(A.elem(2, 3)),   cos(A.elem(2, 3)) * cos(A.elem(2, 1)), - cos(A.elem(2, 3)) * sin(A.elem(2, 1)), A.elem(2, 0) * sin(A.elem(2, 3))},
             {0,                   sin(A.elem(2, 1)),                       cos(A.elem(2, 1)),                     A.elem(2, 2)},
             {0,                   0,                                       0,                                     0}};
    T1[0].matrixCout();
    T1[1].matrixCout();
    T1[2].matrixCout();
    cout << " T2 _______________" << T1.size() << endl;

    array<matrix, 6> T;

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < T.size(); i++)
    {
        T[i] = {{cos(A.elem(i, 3)), -sin(A.elem(i, 3)) * cos(A.elem(i, 1)),   sin(A.elem(i, 3)) * sin(A.elem(i, 1)), A.elem(i, 0) * cos(A.elem(i, 3))},
                {sin(A.elem(i, 3)),  cos(A.elem(i, 3)) * cos(A.elem(i, 1)), - cos(A.elem(i, 3)) * sin(A.elem(i, 1)), A.elem(i, 0) * sin(A.elem(i, 3))},
                {0,                  sin(A.elem(i, 1)),                       cos(A.elem(i, 1)),                     A.elem(i, 2)},
                {0,                  0,                                       0,                                     1}};

    }/*
    for (size_t i = 0; i < T.size(); i++)
    {
        T[i].matrixCout();
        cout << "T[" << i <<"]_________________" << endl;
    }*/

    matrix L = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::ratio<1, 1000000>> durationMcr = stop - start;
    std::chrono::duration<double, std::ratio<1, 1>> durationSec = stop - start;
    cout << "durationMicros = " << durationMcr.count() << endl;
    cout << "durationSec = " << durationSec.count() << endl;
    L.matrixCout();
//    cout << "L1_______________________" << endl;
//    matrix L1 = L * T[2];
//    L1.matrixCout();
//    cout << "L2_______________________" << endl;
//    matrix L2 = L1 * T[3];
//    L2.matrixCout();
//    cout << "L3_______________________" << endl;
    cout << " X = " << L.elem(0, 3) << " Y = " << L.elem(1, 3) << " Z = "<< L.elem(2, 3) << endl;
    cout << " Tet =  " << TO_DEG * acos(L.elem(2,2)) <<
            " Fi  =  " << TO_DEG * acos(- L.elem(1,2) / sin(L.elem(2, 2))) <<
            " Psi =  " << TO_DEG * acos(L.elem(2,1) / sin(L.elem(2, 2))) << endl;




    return 0;
}
