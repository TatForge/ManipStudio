#include "physicslib.h"
#include "solver.h"
#include "robo.h"

struct LibData
{
    unique_ptr<Solver> solver;
    unique_ptr<Robo> manipulator;
};

Angles VectorToAngle(const fvector &v)
{
    Angles a;
    for (counter i = 0; i < JOINTS_COUNT; i++)
    {
        a.q[i]    = v.at(i);
        a.q_p[i]  = v.at(i + JOINTS_COUNT);
        a.q_pp[i] = v.at(i + 2 * JOINTS_COUNT);
    }
    return a;
}
vector<float> AngleToVector(const Angles &a)
{
    vector<float> v;
    v.resize(18);
    for (counter i = 0; i < JOINTS_COUNT; i++)
    {
        v[i + 0 * JOINTS_COUNT] = a.q[i];
        v[i + 1 * JOINTS_COUNT] = a.q_p[i];
        v[i + 2 * JOINTS_COUNT] = a.q_pp[i];
    }
    return v;
}
Matrix4x4 VectorToMatrix4x4(const fvector &v)
{
    Matrix4x4 m;
    for (counter i = 0; i < 4; i++)
    {
        m(0, i) = v[i * 4 + 0];
        m(1, i) = v[i * 4 + 1];
        m(2, i) = v[i * 4 + 2];
        m(3, i) = v[i * 4 + 3];
    }
    return m;
}

PhysicsLib::PhysicsLib()
{
    m_usable = false;
    lib_data = nullptr;
}

PhysicsLib::~PhysicsLib()
{
    delete lib_data;
}

void PhysicsLib::Initialize(){
    delete lib_data;
    lib_data = new LibData;
    lib_data->manipulator = make_unique<Robo>(6);
//    try {
        lib_data->manipulator->LoadManip();
//    }  catch (runtime_error &err) {
//        cout << err.what() << endl;
//    }
    lib_data->solver = make_unique<Solver>(*lib_data->manipulator);
    m_usable = true;
}

void PhysicsLib::MoveJ(const fvector &start_angle, const fvector &end_angle, const float &v_joints, const float &a_joints, fvector &results)
{
    lib_data->solver->MoveJ(VectorToAngle(start_angle), VectorToAngle(end_angle), v_joints, a_joints, 10, results);
}

void PhysicsLib::MoveC(const fvector &start_angle, const fvector &state_vector_p1, const fvector &state_vector_p2, const fvector &state_vector_p3, const float &v_tcp, const float &a_tcp, fvector &results)
{
    vector<float> svp1 = state_vector_p1;
    vector<float> svp2 = state_vector_p2;
    vector<float> svp3 = state_vector_p3;
    //добавить ли контроль размера входного вектора ?
    lib_data->solver->MoveC(VectorToAngle(start_angle), StateVector(svp1[0], svp1[1], svp1[2], ToRad(svp1[3]), ToRad(svp1[4]), ToRad(svp1[5])),
                                                        StateVector(svp2[0], svp2[1], svp2[2], ToRad(svp2[3]), ToRad(svp2[4]), ToRad(svp2[5])),
                                                        StateVector(svp3[0], svp3[1], svp3[2], ToRad(svp3[3]), ToRad(svp3[4]), ToRad(svp3[5])),
                                                        v_tcp, a_tcp, 10, results);

}

void PhysicsLib::MoveL(const fvector &start_angle, const fvector &p1, const fvector &p2, const float &v_tcp, const float &a_tcp, fvector &results)
{
    vector<float> svp1 = p1;
    vector<float> svp2 = p2;
    lib_data->solver->MoveL(StateVector(svp1[0], svp1[1], svp1[2], ToRad(svp1[3]), ToRad(svp1[4]), ToRad(svp1[5])),
                            StateVector(svp2[0], svp2[1], svp2[2], ToRad(svp2[3]), ToRad(svp2[4]), ToRad(svp2[5])),
            v_tcp, a_tcp, VectorToAngle(start_angle), 10, results);
}

void PhysicsLib::DirectKinematic(const fvector &angle, fvector &T)
{
    Matrix4x4 TR[6];
    lib_data->solver->SolveDirectKinematic(VectorToAngle(angle), TR);
    if (T.size() < 6)
        T.resize(6);
    T[0] = TR[5].ConvertToVector3().x;
    T[1] = TR[5].ConvertToVector3().y;
    T[2] = TR[5].ConvertToVector3().z;
    //degrees, like the angles MoveL / MoveC / InverseKinematic take
    Vector3 rpy = TR[5].ConvertToQuaternion().GetEulerAngles();
    T[3] = ToDeg(rpy.x);
    T[4] = ToDeg(rpy.y);
    T[5] = ToDeg(rpy.z);
}

void PhysicsLib::InverseKinematic(const fvector &target, const fvector &curr_angle, fvector &res_angle)
{
    //input
    auto t = target;
    Angles res_angle_a;
    lib_data->solver->SolveInverseKinematic(StateVector(t[0], t[1], t[2], ToRad(t[3]), ToRad(t[4]), ToRad(t[5])), VectorToAngle(curr_angle), res_angle_a);
    res_angle = AngleToVector(res_angle_a);
}

fvector PhysicsLib::GetExternalPosition()
{
    Angles r = lib_data->solver->GetExtarnalPosition();
    fvector result;
    result.resize(6);
    for (uint i = 0; i < JOINTS_COUNT; i++)
    {
        result[i] = r.q[i];
    }
    return result;
}
fvector PhysicsLib::GetInternalPosition()
{
    Angles r = lib_data->solver->GetInternalPosition();
    fvector result;
    result.resize(6);
    for (uint i = 0; i < JOINTS_COUNT; i++)
    {
        result[i] = r.q[i];
    }
    return result;
}
