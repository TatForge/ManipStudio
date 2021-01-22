#ifndef ROBO_H
#define ROBO_H

#include <iostream>

#include "my_math.h"

//#define dt 0.01
#define POSITION_AND_ROTATION
//#define CALC_COUT

#define JOINTS_COUNT 6

#ifdef POSITION_AND_ROTATION
#define EFFECTOR_DEGREE_OF_FREEDOM 6
#else
#define EFFECTOR_DEGREE_OF_FREEDOM 3
#endif

#define RAD M_PI/180
#define DEG 180/M_PI


const scalar damping_constant = 0.01;
const scalar damping_constant_sq = damping_constant * damping_constant;
const scalar MaxAngleDls = ToRad(45.0);

using namespace std;

class Elem
{
public :
    Elem();
    Elem(const RoboMath::Vector3 &pos, const RoboMath::Vector3 &rot, const scalar &theta);
    scalar GetTheta() const {return theta + theta_added;};
    void AddDeltaTheta(const scalar &theta_add);
    inline void SetPositionLocal(const scalar &x, const scalar &y, const scalar &z);
    inline void SetPositionLocal(const RoboMath::Vector3 &pos);
    inline void SetRotationAxisLocal(const RoboMath::Vector3 &rot);
    inline void SetRotationAxis(const RoboMath::Vector3 &rot);
    inline void SetPosition(const RoboMath::Vector3 &V);
//    inline void SetRotation(const RoboMath::Quaternion &rot);
    const RoboMath::Vector3 &GetRotationAxis() const;
    const RoboMath::Vector3 &GetRotationAxisLocal() const;
    const RoboMath::Vector3 &GetPositionLocal() const;
    const RoboMath::Vector3 &GetPosition() const;
//    const RoboMath::Quaternion &GetRotation() const;
//    const RoboMath::Quaternion &GetRotationLocal() const;
protected:
    RoboMath::Vector3 pos;
    RoboMath::Vector3 pos_local;
    RoboMath::Vector3 rotation_axis;
    RoboMath::Vector3 rotation_axis_loc;
//    RoboMath::Quaternion rotation_quat;
//    RoboMath::Quaternion rotation_quat_base;//Положение эффектора, задаваемое на старте программы. Не менять.
    scalar theta;
    scalar theta_added;
};

class Effector : public Elem
{
public:
    inline void SetRotation(const RoboMath::Quaternion &rot);
    const RoboMath::Quaternion &GetRotation() const;
    const RoboMath::Quaternion &GetRotationLocal() const;
private:
    RoboMath::Quaternion rotation_quat;
    RoboMath::Quaternion rotation_quat_base;//Положение эффектора, задаваемое на старте программы. Не менять.
};

class Robo
{
public:
    friend class Emulator;
    struct Moments
    {
        scalar tao[JOINTS_COUNT];
        scalar p[JOINTS_COUNT];
        scalar hikm[JOINTS_COUNT];
        scalar dv[JOINTS_COUNT];
    };
    struct Angles
    {
        scalar q_pp[JOINTS_COUNT];
        scalar q_p[JOINTS_COUNT];
        scalar q[JOINTS_COUNT];
    };
    Robo();
    Robo(int number_of_elements);
    Robo(initializer_list<Elem> &robo_list);
    ~Robo();
    void RoboCout();
    void CalculateJacobian();
    void CalculateDeltaThetas();
    void CalculateCurrentPosition();
    void CalculateDeltaS(RoboMath::Vector3 target);
    void CalculateDeltaR(RoboMath::Vector3 target_r);
    void AddDeltaThetas();
    void ComputeJointsPosition();
    void SolveDirectKinematic(const scalar *tet, RoboMath::Matrix * const state_matrix);
    void SolveInverseKinematic();
    void DeltaThetasCout();
    void SolveDirectDynamic();
    void SolveInverseDynamic();
    void CalcPolynomLine(const Angles *angle_start, const Angles *angle_stop, const scalar t, RoboMath::Polynom *result);
    void CalcPolynomCurve(const Angles *angle_0, const Angles *angle_1, const Angles *angle2, const Angles *angle3);
    Moments *GetMoments();
    Angles *GetAngles();
    void SimulateManipPolinoms(RoboMath::Polynom *pol, const scalar t);


    RoboMath::Vector3 GetEffectorPosition() const;
    RoboMath::Vector3 GetEffectorRotation() const;
    RoboMath::Quaternion GetEffectorRotationQuat() const;
    RoboMath::Vector3 GetDeltaR() const;
    RoboMath::Vector3 GetDeltaS() const;
    const Elem *GetEffector() const;

    const Elem *GetElem(int number) const;

    //maybe trash

private:
    Elem *robot;
    RoboMath::Matrix *jacobian;
    RoboMath::Matrix4x4 T[JOINTS_COUNT];
    RoboMath::Vector *delta_thetas;
    RoboMath::Vector3 delta_s;
    RoboMath::Vector3 delta_r;
    Effector effector;

    scalar m_mass[JOINTS_COUNT];

    Angles m_angles;
    Moments m_moments;

    RoboMath::Matrix4x4 U[36];
    array<RoboMath::Vector, JOINTS_COUNT> m_cm;
    array<RoboMath::Tensor, JOINTS_COUNT> m_i;
    array<RoboMath::Matrix4x4, JOINTS_COUNT> m_ji;

    RoboMath::Matrix *state_matrix;
    RoboMath::Vector *theta_vector;

    void RoboSettingsInit();
    void CalcUMatrix();
    int number_of_elements;
    float dt;
    void AllocateForNewRobot();
};



#endif // ROBO_H
