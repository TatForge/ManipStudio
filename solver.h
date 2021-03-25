#ifndef SOLVER_H
#define SOLVER_H
#include "robo.h"
#include "emulator.h"
#include "meshcollider.h"
//#include "gjkcollision3d.h"
//#include "my_math.h"



using namespace rm;

struct BoxAABB
{
    BoxAABB(const scalar &x_min, const scalar &y_min, const scalar &z_min, const scalar &x_max, const scalar &y_max, const scalar &z_max);
    Vector3 point_max;
    Vector3 point_min;
};

class Solver
{
public:
    Solver(Robo &manip);
    void SolveDirectKinematic(const Angles &q, Matrix4x4 TR[]);
    void SolveInverseKinematic(const scalar &accuracy, const StateVector &target, const Angles &curr_angle, Angles &res_angle);
    int CalcPolynomMaxQPP(const StateVector &start_point, const StateVector &end_point, const Angles &curr_angles, Angles& res_angle, const int &t_millisec, const scalar &dt_millisec, Polynom result[]);
    void CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const Angles &start_angles, Angles &res_angle, const ushort &dt_millisec, Polynom result[]);
//    int CalcPolynomCircle(const StateVector &start_point, const StateVector &end_point,)
    bool CheckTrajectoryForLinear(const Polynom &pol, const Vector3 &l1, const Vector3 &l2, const scalar &t_millisec, const scalar &max_delta);
    bool CheckTrajectoryForLinear(const Angles angles, const Vector3 &l1, const Vector3 &l2, const scalar &max_delta);
    bool /*tuple<vector<Vector3>, vector<Quaternion>> */MoveMeshes(const Matrix4x4 TR[])
    {
        std::vector<Vector3>v_move;
        std::vector<Quaternion> quat_move;
        v_move.push_back(Vector3(0, 0, 0));
        v_move.push_back(TR[0].ConvertToVector3());
        v_move.push_back(TR[1].ConvertToVector3());
        v_move.push_back(TR[2].ConvertToVector3());
        v_move.push_back(TR[3].ConvertToVector3());
        v_move.push_back(TR[4].ConvertToVector3());
        v_move.push_back(TR[5].ConvertToVector3());

        quat_move.push_back(TR[0].ConvertToQuaternion());
        quat_move.push_back(TR[1].ConvertToQuaternion());
        quat_move.push_back(TR[2].ConvertToQuaternion());
        quat_move.push_back(TR[3].ConvertToQuaternion());
        quat_move.push_back(TR[4].ConvertToQuaternion());
        quat_move.push_back(TR[5].ConvertToQuaternion());

        //пересчет точек тут же
        meshes_buff = meshes;

        meshes_buff[0].MoveMesh(quat_move[0], Vector3(0, 0, 0));
        meshes_buff[1].MoveMesh(quat_move[0], v_move[1]);
        meshes_buff[2].MoveMesh(quat_move[1], v_move[2]);
        meshes_buff[3].MoveMesh(quat_move[2], v_move[3]);
        meshes_buff[4].MoveMesh(quat_move[2], v_move[3]);
        meshes_buff[5].MoveMesh(quat_move[3], v_move[4]);
        meshes_buff[6].MoveMesh(quat_move[4], v_move[4]);
        meshes_buff[7].MoveMesh(quat_move[5], v_move[5]);
        return true;
    };
//    void MoveMeshes(const Quaternion qtr[], const Vector3 v[]);
    pair<int, int> CheckCollisionAABB(const Matrix4x4 state[]);

private:
    Solver(Solver &solver) = delete;
    Solver(Solver &&solver) = delete;
    bool CheckAABB(const BoxAABB &box1, const BoxAABB &box2);
    void CalculateDeltas();
    const Matrix *manip_config;         //Матрица хранит конфигурацию манипулятора, не текущее состояние
    Matrix4x4 T[JOINTS_COUNT];          //buff matrix for calculations
    Emulator *emulator;
    std::vector<MeshCollider> meshes;
    std::vector<MeshCollider> meshes_buff;

//    GJKCollision *collider;
//    Matrix4x4 T0[JOINTS_COUNT];
};

#endif // SOLVER_H
