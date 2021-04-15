#ifndef SOLVER_H
#define SOLVER_H
#include "robo.h"
#include "emulator.h"
#include "meshcollider.h"

#include <condition_variable>
#include <queue>

using namespace rm;

struct BoxAABB
{
    BoxAABB(const scalar &x_min, const scalar &y_min, const scalar &z_min, const scalar &x_max, const scalar &y_max, const scalar &z_max);
    Vector3 point_max;
    Vector3 point_min;
};

struct IKData
{
    Quaternion end_eff_target_ori;
    Quaternion end_eff_world_ori;
    Quaternion deltaQ;
    Matrix4x4 TR[6];
    Vector3 ang_vel;
    Vector3 delta_r;
    Vector3 delta_s;
    Vector3 axis;
    Vector3 maxis;
    Matrix jacobian = Matrix(6,6);
    Matrix jacobian_transposed = Matrix(6, 6);
    Matrix U = Matrix(6, 6);
    Matrix aug_matrix = Matrix(6, 7);
    Angles d_thet;
    Vector delta_t = Vector(6);
    Vector delta_thetas = Vector(6);
    counter ctr = 0;
    scalar angle = 0;
};


class Solver
{
public:
    Solver(Robo &manip);
    void SolveDirectKinematic(const Angles &q, Matrix4x4 TR[], Matrix4x4 *T = nullptr) noexcept;
    void SolveInverseKinematic(const scalar &accuracy, const StateVector &target, const Angles &curr_angle, Angles &res_angle, IKData *data  = nullptr);
    void CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const Angles &start_angles, Angles &res_angle, const ushort &dt_millisec, Polynom result[]);
    bool CheckTrajectoryForLinear(const Polynom &pol, const Vector3 &l1, const Vector3 &l2, const scalar &t_millisec, const scalar &max_delta);
    bool CheckTrajectoryForLinear(const Angles angles, const Vector3 &l1, const Vector3 &l2, const scalar &max_delta);
    bool MoveMeshes(const Matrix4x4 TR[]) noexcept;
    void CalcUMatrix() noexcept;
    //Multithread
    void SolveForwardDynamicMultithread();
    void CheckCollisionMultithread();
    bool SolveForwardDynamic(const Angles &angles);
private:
    Solver(Solver &solver) = delete;
    Solver(Solver &&solver) = delete;
    bool CheckAABB(const BoxAABB &box1, const BoxAABB &box2);
    void CalculateDeltas();
    const Matrix *manip_config;         //Матрица хранит конфигурацию манипулятора, не текущее состояние
    Matrix4x4 m_T[JOINTS_COUNT];          //buff matrix for calculations
    Emulator *m_emulator;
    std::vector<MeshCollider> m_meshes;
    std::vector<MeshCollider> m_meshes_buff;
    //Dynamic___________________
    Robo::Moments m_moments;
    array<Tensor,6> m_inertion;
    std::vector<scalar> m_mass;
    Matrix4x4 U[36];
    queue<exception_ptr> exception_queue;
    Vector m_gv{0, 0, -g, 0};
    array<rm::Vector, JOINTS_COUNT> m_cm;
    array<rm::Tensor, JOINTS_COUNT> m_i;
    array<rm::Matrix4x4, JOINTS_COUNT> m_ji;

    vector<Angles>  m_angles_v;
    vector<Vector3> m_speed_v;
    vector<Vector3> m_accel_v;
    vector<Robo::Moments> m_moments_vector;
    uint m_moments_counter;
    uint m_collisions_counter;
    IKData *m_main_IKData;

    //    Angles m_buff_angles_last;
    //    Angles m_buff_angles_previous;
#if MULTITHREAD
    IKData *m_dynamic_IKData;
    thread *m_dynamic_thread;
    thread *m_collision_thread;
    thread *m_visual_thread;
    condition_variable m_dynamic_cv;
    condition_variable m_collision_cv;
    condition_variable m_visual_cv;
    bool m_dynamic_notified;
    bool m_collision_notified;
    bool m_visual_notified;
    mutex m_dynamic_mtx;
    mutex m_collision_mtx;
    mutex m_visual_mtx;
#endif
    //__________________________
//    GJKCollision *collider;
//    Matrix4x4 T0[JOINTS_COUNT];
};

#endif // SOLVER_H
