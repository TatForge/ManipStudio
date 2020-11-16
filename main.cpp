#include <main.h>
#include <array>
#include <chrono>
#include <string.h>
#include <thread>

#include "robo.h"
//#include "commandworker.h"


using namespace RoboMath;

int main(int argc, char *argv[])
{

//    array<scalar, 6> tet {0 * M_PI/180,
//                         0 * M_PI/180,
//                         0 * M_PI/180,
//                         0 * M_PI/180,
//                         0 * M_PI/180,
//                         0 * M_PI/180};

//    Matrix A {{0,   -M_PI/2, 232.5,  tet[0]},
//              {375, 0,       0,      tet[1]},
//              {295, 0,       0,      tet[2]},
//              {0,   -M_PI/2, 120.5,  tet[3]},
//              {0,   M_PI/2,  171,    tet[4]},
//              {0,   0,       122.6,  tet[5]}};
    Vector3 target(0.7, 0.1, 0.1);
//    Matrix Jacobian(3, 5);
//    scalar dampingLambda = 0.01;
//    scalar DLS = dampingLambda * dampingLambda;
    initializer_list<Elem> il_elem {Elem(Vector3(0,     0,      0),      unit_z,  0 * RAD),
                                    Elem(Vector3(0,     0,      0.2325), unit_y,  0 * RAD),
                                    Elem(Vector3(0.375, 0,      0.2325), unit_y,  0 * RAD),
                                    Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  0 * RAD),
                                    Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, 0 * RAD),
                                    Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  0 * RAD)};
    Robo manipulator(il_elem);

    for (int i = 0; i < 100; i ++)
    {

        bool solved = true;
//        target = Vector3(abs(sin(i)) * 0.5, abs(cos(i)) * 0.2, abs(cos(i)) * 0.1);
        target = Vector3(sin(i) * 0.3, cos(i) * 0.2, cos(i) * 0.1);
//        target = Vector3(0.0705, -0.198, -0.099);
        Vector3 delta = target - manipulator.GetEffectorPosition();
        counter ctr(0);
        checkTime ct;
        while(delta.Length() > 10e-5)
        {
    //        manipulator.SetConfigurationMatrix(A);
    //        manipulator.RoboCout();
            manipulator.CalculateJacobian();

    //        qDebug() << "2";
            manipulator.CalculateDeltaS(target);//RoboMath::Vector3({0.1, 0.1, 0.4}));
            manipulator.CalculateDeltaThetas();
//            manipulator.DeltaThetasCout();

    //        qDebug() << "3";
            manipulator.AddDeltaThetas();
            manipulator.ComputeJointsPosition();
            manipulator.UpdateJointsLocalPosition();
//            cout << "Effector position "
//                 << " x = " << manipulator.GetEffectorPosition().x
//                 << " y = " << manipulator.GetEffectorPosition().y
//                 << " z = " << manipulator.GetEffectorPosition().z
//                 << endl;

    //        qDebug() << "4";
    //        manipulator.RoboCout();
            delta = target - manipulator.GetEffectorPosition();
            ctr++;
            if (ctr > 10000)
            {
                cout << "Уравнение не сошлось !_!_!_!_!_!_!_!_!_!_!_!_!" << endl;
                cout << "Положение эффектора " << endl
                     << " x = " << manipulator.GetEffectorPosition().x
                     << " y = " << manipulator.GetEffectorPosition().y
                     << " z = " << manipulator.GetEffectorPosition().z
                     << endl ;
                cout << " Положение цели "
                     << " x = " << target.x
                     << " y = " << target.y
                     << " z = " << target.z
                     << endl;
                solved = false;
                break;
            }
        }

        auto d = ct.duration().count();
        if (solved)
        {
            cout << "Уравнение сошлось за " << ctr << " решений" << endl;
            cout << "Положение эффектора " << endl
                 << " x = " << manipulator.GetEffectorPosition().x
                 << " y = " << manipulator.GetEffectorPosition().y
                 << " z = " << manipulator.GetEffectorPosition().z
                 << endl ;
            cout << " Положение цели "
                 << " x = " << target.x
                 << " y = " << target.y
                 << " z = " << target.z
                 << endl;
        }


        cout << "Время расчета составило " << d << " секунд" << endl << endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }




    //matrixMultiplyTranspose
    //doing transpose J matrix

//    Matrix JacobianTransposed = Jacobian;
//    JacobianTransposed = Jacobian;
//    JacobianTransposed.Transpose();
//    Matrix U = Jacobian * JacobianTransposed;
//    U.AddDiagonal(DLS);
//    //Solving U matrix
//    Matrix AM(3, 4);
//    Vector dS{target.x - manip[5].GetPosition().x, target.y - manip[5].GetPosition().y, target.y - manip[5].GetPosition().z};
////        *(target + 0) - manip[5].GetPosition().x, *(target + 1) - manip[5].GetPosition().y, *(target + 2) - manip[5].GetPosition().z};
//    AM.SetSubmatrix(U);
//    AM.SetColumn(dS, 4);
//    AM.SolveGauss();
//    Vector dT(3);
//    AM.GetTheRoots(dT);
//    for (int i = 0; i < dT.GetLength(); i++)
//    {
//        cout << "dT " << *(dT.GetDataPtr() + i) << endl;
//    }
//    Vector dTet(5);
//    dTet = JacobianTransposed * dT;


    return 0;
}

