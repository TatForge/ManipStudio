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
    Solver(Robo &robot);
    ~Solver()
    {
        delete manip_config;
        delete m_main_IKData;
        delete m_emulator;

#if MULTITHREAD
        delete m_dynamic_IKData;
        m_closing = 1;
        m_collision_cv.notify_all();
        m_dynamic_cv.notify_all();
#endif
    }
    void SolveDirectKinematic(const Angles &q, Matrix4x4 TR[], Matrix4x4 *T = nullptr) noexcept;
    void SolveInverseKinematic(const StateVector &target, const Angles &curr_angle, Angles &res_angle, IKData *data  = nullptr);
    void MoveL(const StateVector &start_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const Angles &start_angles, const ushort &dt_millisec, vector<float> &results);
    void MoveJ(const Angles &start_angle, const Angles &end_angle, const float &speed, const float &acceleration, const float &dt_millisec, vector<float> &results);;
    bool MoveC(const Angles &start_angle, const StateVector &start_point, const StateVector &mid_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const scalar &dt_millisec, vector<float> &results);
    bool CheckTrajectoryForLinear(const Polynom &pol, const Vector3 &l1, const Vector3 &l2, const scalar &t_millisec, const scalar &max_delta);
    bool CheckTrajectoryForLinear(const Angles angles, const Vector3 &l1, const Vector3 &l2, const scalar &max_delta);
    bool MoveMeshes(const Matrix4x4 TR[]) noexcept;
    void CalcUMatrix() noexcept;
    void CalcUMatrix(const Angles &a);
    //Multithread
    void SolveForwardDynamicMultithread();
    void CheckCollisionMultithread();
    bool SolveForwardDynamic(const Angles &angles);
    void VisualizeMoments(const scalar &dt_millisec);
    inline void AnglesVectorToVector(const int size, vector<float> &results);

    int CalcTrajOneJoint(const Angles &start_angle, const Angles &end_angle, const scalar &dt_s, const scalar &angle_max_speed, const scalar &angle_acceleration, const int &element_number);
    void VectorsResize(const counter &ctr);
    void CheckCollision(const uint *elem_number = nullptr);
    void CalcCollisionsDynamics(const scalar &array_size, const scalar &dt_millisec);
    inline scalar FindAngle(const Vector3 &O, const Vector3 &A0, const Vector3 P);

private:
    Solver(Solver &solver) = delete;
    Solver(Solver &&solver) = delete;
    bool CheckAABB(const BoxAABB &box1, const BoxAABB &box2);
    void CalculateDeltas();

    std::vector<MeshCollider> m_meshes_buff;
    std::vector<MeshCollider> m_meshes;
    Matrix4x4 m_T[JOINTS_COUNT];
    const Matrix *manip_config;
    Emulator *m_emulator;

    //Dynamic___________________
    queue<exception_ptr> exception_queue;
    array<Tensor,6> m_inertion;
    std::vector<scalar> m_mass;
    Vector m_gv{0, 0, -g, 0};
    Robo::Moments m_moments;
    Matrix4x4 U[36];
    array<rm::Matrix4x4, JOINTS_COUNT> m_ji;
    array<rm::Vector, JOINTS_COUNT> m_cm;
    array<rm::Tensor, JOINTS_COUNT> m_i;
    vector<scalar> max_joint_moments[6];
    vector<Angles>  m_angles_v;
    vector<Vector3> m_speed_v;
    vector<Vector3> m_accel_v;
    vector<Robo::Moments> m_moments_v;
    uint m_collisions_counter;
    uint vector_size = 500;
    uint m_moments_counter;
    IKData *m_main_IKData;

#if MULTITHREAD
    condition_variable m_collision_cv;
    condition_variable m_dynamic_cv;
    IKData *m_dynamic_IKData;
    thread *m_collision_thread;
    thread *m_dynamic_thread;
    thread *m_visual_thread;
    bool m_collision_notified;
    bool m_dynamic_notified;
    bool m_visual_notified;
    mutex m_collision_mtx;
    mutex m_dynamic_mtx;
    mutex m_visual_mtx;
    bool m_closing;
#endif
};

#endif // SOLVER_H
