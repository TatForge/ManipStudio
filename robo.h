#ifndef ROBO_H
#define ROBO_H

#include <iostream>

#include "my_math.h"

//#define dt 0.01
#define POSITION_AND_ROTATION
//#define CALC_COUT

#ifdef POSITION_AND_ROTATION
#define EFFECTOR_DEGREE_OF_FREEDOM 6
#else
#define EFFECTOR_DEGREE_OF_FREEDOM 3
#endif

#define RAD M_PI/180
#define DEG 180/M_PI

inline static scalar ToRad(scalar degrees) {return degrees * RAD;};
inline static scalar ToDeg(scalar radians) {return radians * DEG;};
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
    struct Moments
    {
        scalar tao;
        scalar p;
        scalar hikm;
        scalar dv;
    };
    struct Angles
    {
        scalar q_pp;
        scalar q_p;
        scalar q;
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
    void DeltaThetasCout();
    void SolveDirectDynamic();
    void SolveInverseDynamic();
    void CalcPolynomLine(const Angles *angle_start, const Angles *angle_stop, const scalar t0, const scalar t1, RoboMath::Polynom *result);
    Moments *GetMoments();
    Angles *GetAngles();


    RoboMath::Vector3 GetEffectorPosition() const;
    RoboMath::Vector3 GetEffectorRotation() const;
    RoboMath::Quaternion GetEffectorRotationQuat() const;
    RoboMath::Vector3 GetDeltaR() const;
    RoboMath::Vector3 GetDeltaS() const;
    const Elem *GetEffector() const;

private:
    Elem *robot;
    RoboMath::Matrix *jacobian;
    RoboMath::Matrix4x4 T[6];
    RoboMath::Vector *delta_thetas;
    RoboMath::Vector3 delta_s;
    RoboMath::Vector3 delta_r;
    Effector effector;

    scalar m_mass[6];

    Angles m_angles[6];
    Moments m_moments[6];

    RoboMath::Matrix4x4 U[36];
    array<RoboMath::Vector, 6> m_cm;
    array<RoboMath::Tensor, 6> m_i;
    array<RoboMath::Matrix4x4, 6> m_ji;



    //moments calculate methods
    void CalcHjkm();
//    const RoboMath::Matrix* state_matrix;
//    const RoboMath::Vector* theta_vector;
    void RoboSettingsInit();
    void CalcUMatrix();
    int number_of_elements;
    float dt;
    void AllocateForNewRobot();
};



#endif // ROBO_H
