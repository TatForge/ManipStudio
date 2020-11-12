#include <main.h>
#include <qdebug.h>
#include <array>
#include <chrono>
#include <string.h>

#include "robo.h"


using namespace RoboMath;

int main(int argc, char *argv[])
{
    array<scalar, 6> tet {0 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180,
                         0 * M_PI/180};

    Matrix A {{0,   -M_PI/2, 232.5,  tet[0]},
              {375, 0,       0,      tet[1]},
              {295, 0,       0,      tet[2]},
              {0,   -M_PI/2, 120.5,  tet[3]},
              {0,   M_PI/2,  171,    tet[4]},
              {0,   0,       122.6,  tet[5]}};


//    scalar *target = new scalar [3] {0.1, 0.1, 0.4};
    Vector3 target(0.7, 0.1, 0.1);

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
                                   {Elem(Vector3(0,     0,      0.2325), unit_y,  0 * RAD)},
                                   {Elem(Vector3(0.375, 0,      0.2325), unit_y,  0 * RAD)},
                                   {Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  0 * RAD)},
                                   {Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, 0 * RAD)},
                                   {Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  0 * RAD)}};
    Vector3 delta = target - manip[5].GetPosition();
    Robo manipulator(il_elem);
    while(delta.Length() > 10e-5)
    {
//        manipulator.SetConfigurationMatrix(A);
        manipulator.RoboCout();
        manipulator.CalculateJacobian();
        manipulator.CalculateDeltaS(target);//RoboMath::Vector3({0.1, 0.1, 0.4}));
        manipulator.CalculateDeltaThetas();
        manipulator.DeltaThetasCout();
        manipulator.AddDeltaThetas();
        manipulator.ComputeJointsPosition();
        manipulator.UpdateJointsLocalPosition();

        manipulator.RoboCout();
        delta = target - manipulator.GetEffectorPosition();
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
    Vector dS{target.x - manip[5].GetPosition().x, target.y - manip[5].GetPosition().y, target.y - manip[5].GetPosition().z};
//        *(target + 0) - manip[5].GetPosition().x, *(target + 1) - manip[5].GetPosition().y, *(target + 2) - manip[5].GetPosition().z};
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


    return 0;
}

