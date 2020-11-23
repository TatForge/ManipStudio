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

    //_---------------------------------------------
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

//    array<Matrix, 6> T;
//    for (counter i = 0; i < T.size(); i++)
//    {
//        T[i] = {{cos(A.elem(i, 3)), -sin(A.elem(i, 3)) * cos(A.elem(i, 1)),   sin(A.elem(i, 3)) * sin(A.elem(i, 1)), A.elem(i, 0) * cos(A.elem(i, 3))},
//                {sin(A.elem(i, 3)),  cos(A.elem(i, 3)) * cos(A.elem(i, 1)), - cos(A.elem(i, 3)) * sin(A.elem(i, 1)), A.elem(i, 0) * sin(A.elem(i, 3))},
//                {0,                  sin(A.elem(i, 1)),                       cos(A.elem(i, 1)),                     A.elem(i, 2)},
//                {0,                  0,                                       0,                                     1}};

//    }

    //_---------------------------------------------


    Vector3 target(0.7, 0.1, 0.1);
    Vector3 target_r(ToRad(20), ToRad(30), ToRad(40));//Elem(Vector3(0,     0,      0),      Vector3(0,0,0), ToRad(0)),
    initializer_list<Elem> il_elem {Elem(Vector3(0,     0,      0),      unit_z,  ToRad(0)),
                                    Elem(Vector3(0,     0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.375, 0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, ToRad(0)),
                                    Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  ToRad(0))};
    Robo manipulator(il_elem);
    checkTime ct;
    for (int i = 0; i < 10000; i ++)
    {
        bool solved = true;
//        target = Vector3(sin(i) * 0.3, cos(i) * 0.2, cos(i) * 0.1);
        target = Vector3(0.3, 0.2, 0.1);
        Vector3 delta = target - manipulator.GetEffectorPosition();
        counter ctr(0);
//        checkTime ct;
        while(delta.Length() > 10e-5)
        {
            manipulator.CalculateJacobian();
            manipulator.CalculateDeltaS(target);
            manipulator.CalculateDeltaR(target_r);
            manipulator.CalculateDeltaThetas();
            manipulator.AddDeltaThetas();
            manipulator.ComputeJointsPosition();
            manipulator.UpdateJointsLocalPosition();
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

//        auto d = ct.duration().count();
//        if (solved)
//        {
//            cout << "Уравнение сошлось за " << ctr << " решений" << endl;
//            cout << "Положение эффектора " << endl
//                 << " x = " << manipulator.GetEffectorPosition().x
//                 << " y = " << manipulator.GetEffectorPosition().y
//                 << " z = " << manipulator.GetEffectorPosition().z
//                 << endl ;
//            cout << " Положение цели "
//                 << " x = " << target.x
//                 << " y = " << target.y
//                 << " z = " << target.z
//                 << endl;
//        }
//        cout << "Время расчета составило " << d << " секунд" << endl << endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    cout << "Time of 100000 calculations " << ct.duration().count() << endl;
    cout << "One iter duration = " << ct.duration().count() / 10000.0 << endl;
    return 0;
}

