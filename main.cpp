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

    initializer_list<Elem> il_elem {Elem(Vector3(0,     0,      0),      unit_z,  ToRad(0)),
                                    Elem(Vector3(0,     0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.375, 0,      0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.2325), unit_y,  ToRad(0)),
                                    Elem(Vector3(0.670, 0.1205, 0.0615), unit_mz, ToRad(0)),
                                    Elem(Vector3(0.670, 0.2431, 0.0615), unit_y,  ToRad(0))};
    Robo manipulator(il_elem);
    int *a = new int;
    manipulator.ComputeJointsPosition();
    int *i = new int;
    checkTime ct;
    int kolvo = 100;
    for (int i = 0; i < kolvo; i ++)
    {
        bool solved = true;
//        Vector3 target(sin(i) * 0.3, cos(i) * 0.2, cos(i) * 0.1);
        Vector3 target(0.67, 0.2431, 0.0615);
        Vector3 target_r(ToRad(90), ToRad(30), ToRad(0));
//        Vector3 target_r(sin(i) * ToRad(90), cos(i) * ToRad(80), cos(i) * ToRad(-30));
        cout << " x = " << manipulator.Effector()->GetPosition().x
             << " y = " << manipulator.Effector()->GetPosition().y
             << " z = " << manipulator.Effector()->GetPosition().z
             << endl;
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
                     << " x = " << manipulator.GetEffectorRotation().x
                     << " y = " << manipulator.GetEffectorRotation().y
                     << " z = " << manipulator.GetEffectorRotation().z
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
        if (solved)
        {
            cout << "Уравнение сошлось за " << ctr << " решений" << endl;
            cout << "Положение эффектора " << endl
                 << " ang1 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().x)
                 << " ang2 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().y)
                 << " ang3 = " << ToDeg(manipulator.GetEffectorRotationQuat().GetEulerAngles().z)
                 << endl ;
            cout << " Положение цели "
                 << " x = " << ToDeg(target_r.x)
                 << " y = " << ToDeg(target_r.y)
                 << " z = " << ToDeg(target_r.z)
                 << endl;
        }
//        cout << "Время расчета составило " << d << " секунд" << endl << endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    cout << "Time of " << kolvo << " calculations " << ct.duration().count() << endl;
    cout << "One iter duration = " << ct.duration().count() / kolvo << endl;
    cout << "Точность " << manipulator.GetDeltaR().Length() << endl;
    return 0;
}

