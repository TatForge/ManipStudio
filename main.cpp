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

    array<Matrix, 6> T;
    for (counter i = 0; i < T.size(); i++)
    {
        T[i] = {{cos(A.at(i, 3)), -sin(A.at(i, 3)) * cos(A.at(i, 1)),   sin(A.at(i, 3)) * sin(A.at(i, 1)), A.at(i, 0) * cos(A.at(i, 3))},
                {sin(A.at(i, 3)),  cos(A.at(i, 3)) * cos(A.at(i, 1)), - cos(A.at(i, 3)) * sin(A.at(i, 1)), A.at(i, 0) * sin(A.at(i, 3))},
                {0,                  sin(A.at(i, 1)),                       cos(A.at(i, 1)),                     A.at(i, 2)},
                {0,                  0,                                       0,                                     1}};

    }
    Matrix L = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
    L.MatrixCout();
//    cout << " Tet =  " << DEG * acos(L.elem(2,2)) <<
//            " Fi  =  " << DEG * acos(- L.elem(1,2) / sin(L.elem(2, 2))) <<
//            " Psi =  " << DEG * acos(L.elem(2,1) / sin(L.elem(2, 2))) << endl;
    auto k = L(2,0);
    auto ke = L(2,1);
    double fi = atan(L(2,0)/L(2,1));
    double tet1 = atan(sin(fi)*L(2,0) - cos(fi)*L(2,1)/L(2,2));
    double psi = atan(L(0,2)/L(1,2));
    //psi = yaw || fi = pitch || tet = roll
    cout << " x =  " << L(0,3) <<
            " y =  " << L(1,3) <<
            " z =  " << L(2,3) << endl;
    cout << " Roll =  " << ToDeg(tet1) <<
            " Pitch  =  " << ToDeg(fi) <<
            " Yaw =  " << ToDeg(psi) << endl;

    //_---------------------------------------------


    Vector3 target(0.7, 0.1, 0.1);
    Vector3 target_r(ToRad(90), ToRad(0), ToRad(0));//Elem(Vector3(0,     0,      0),      Vector3(0,0,0), ToRad(0)),
    initializer_list<Elem> il_elem {Elem(Vector3(0,     0,      0),      unit_z,  ToRad(0)),
                                    Elem(Vector3(0,     0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.375, 0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, ToRad(0)),
                                    Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  ToRad(0))};
    Robo manipulator(il_elem);
//    manipulator.
    manipulator.SetConfigurationMatrix(A);


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
            //denavita____________________________
            manipulator.UpdateThetaValues();
            manipulator.UpdateTranslationMatrix();
            //____________________________________
            manipulator.CalculateJacobian();
            manipulator.CalculateDeltaS(target);
            manipulator.CalculateDeltaR(target_r);
            manipulator.CalculateDeltaThetas();
            manipulator.AddDeltaThetas();
            //denavita____________________________
            manipulator.UpdateThetaValues();
            manipulator.UpdateTranslationMatrix();
            //____________________________________
            manipulator.ComputeJointsPosition();
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

