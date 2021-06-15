#include "physicslib.h"
#include "solver.h"
#include "emulator.h"
#include "robo.h"

struct LibData
{
    Solver *solver;
    Emulator emulator;
    Robo manipulator;
};

Angles VectorToAngle(const fvector &v)
{
    Angles a;
    for (counter i = 0; i < JOINTS_COUNT; i++)
    {
        a.q[i]    = v.at(i);
        a.q_p[i]  = v.at(i + JOINTS_COUNT);
        a.q_pp[i] = v.at(i + JOINTS_COUNT);
    }
    return a;

}

PhysicsLib::PhysicsLib()
{
    try {
        lib_data->manipulator.LoadManip();
    }  catch (runtime_error &err) {
        cout << err.what() << endl;
    }
    lib_data->solver = new Solver(lib_data->manipulator);

}

void PhysicsLib::MoveJ(const fvector &start_angle, const fvector &end_angle, const float &v_tcp, const float &a_tcp, fvector &results)
{
    lib_data->solver->MoveJ(VectorToAngle(start_angle), VectorToAngle(end_angle), v_tcp, a_tcp, 10, results);
}

void PhysicsLib::MoveC(const fvector &start_angle, const fvector &state_vector_p1, const fvector &state_vector_p2, const fvector &state_vector_p3, const float &v_tcp, const float &a_tcp, fvector &results)
{
    vector<float> svp1 = state_vector_p1;
    vector<float> svp2 = state_vector_p2;
    vector<float> svp3 = state_vector_p3;
    lib_data->solver->MoveCircle(VectorToAngle(start_angle), StateVector(svp1[0], svp1[1], svp1[2], ToRad(svp1[3]), ToRad(svp1[4]), ToRad(svp1[5])),
                                                             StateVector(svp2[0], svp2[1], svp2[2], ToRad(svp2[3]), ToRad(svp2[4]), ToRad(svp2[5])),
                                                             StateVector(svp3[0], svp3[1], svp3[2], ToRad(svp3[3]), ToRad(svp3[4]), ToRad(svp3[5])),
                                                             v_tcp, a_tcp, 10, results);

}

void PhysicsLib::MoveL(const fvector &start_angle, const fvector &state_vector_p1, const fvector &state_vector_p2, const float &v_tcp, const float &a_tcp, fvector &results)
{
    vector<float> svp1 = state_vector_p1;
    vector<float> svp2 = state_vector_p2;
    lib_data->solver->MoveL(StateVector(svp1[0], svp1[1], svp1[2], ToRad(svp1[3]), ToRad(svp1[4]), ToRad(svp1[5])),
                            StateVector(svp2[0], svp2[1], svp2[2], ToRad(svp2[3]), ToRad(svp2[4]), ToRad(svp2[5])),
                            v_tcp, a_tcp, VectorToAngle(start_angle), 10, results);
}
