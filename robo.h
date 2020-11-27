#ifndef ROBO_H
#define ROBO_H

#include <iostream>

#include "my_math.h"

#define POSITION_AND_ROTATION

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
    inline void setRotationLocal(const RoboMath::Vector3 &rot);
    inline void SetRotation(const RoboMath::Vector3 &rot);
    inline void SetPos(RoboMath::Vector3 &V);
    const RoboMath::Vector3 &GetRotationAxis() const;
    const RoboMath::Vector3 &GetRotationAxisLocal() const;
    const RoboMath::Vector3 &GetPositionLocal() const;
    const RoboMath::Vector3 &GetPosition() const;
private:
    RoboMath::Vector3 pos;
    RoboMath::Vector3 pos_local;
    RoboMath::Vector3 rotation_axis;
    RoboMath::Vector3 rotation_axis_loc;
    scalar theta;
    scalar theta_added;
};

class Robo
{
public:
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
    RoboMath::Vector3 GetEffectorPosition() const;
    RoboMath::Vector3 GetEffectorRotation() const;
    RoboMath::Quaternion GetEffectorRotationQuat() const;
    RoboMath::Vector3 GetDeltaR() const;
    RoboMath::Vector3 GetDeltaS() const;
    const Elem *Effector() const;
private:
    Elem *robot;
    RoboMath::Matrix *jacobian;
    RoboMath::Vector *delta_thetas;
    RoboMath::Vector3 delta_s;
    RoboMath::Vector3 delta_r;
    RoboMath::Quaternion effector_rot;
    RoboMath::Quaternion effector_rot_base;
    int number_of_elements;
    void AllocateForNewRobot();
};



#endif // ROBO_H
