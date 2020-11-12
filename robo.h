#ifndef ROBO_H
#define ROBO_H

#include <iostream>

#include "my_math.h"

//#define POSITION_AND_ROTATION

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
    Elem(const RoboMath::Vector3 pos, const RoboMath::Vector3 rot, const scalar theta);

    RoboMath::Vector ConvertToVector();

    scalar GetTheta() const {return theta + theta_added;};
    const RoboMath::Vector3 GetRotationAxis() const;
    const RoboMath::Vector3 GetPosLocal() const;
    const RoboMath::Vector3 GetPosition() const;

    void AddDeltaTheta(const scalar &theta_add);;
    inline void SetPos(RoboMath::Vector3 &V);

    inline void SetRotation(const RoboMath::Vector3 &rot);
    inline void SetLocalPos(const RoboMath::Vector3 &pos);
    inline void SetLocalPos(const scalar &x, const scalar &y, const scalar &z)
    {
        pos_local = RoboMath::Vector({x, y, z});
    }

    void elemCout()
    {
//        cout << "X = " << x_pos << " Y = " << y_pos << " Z = " << z_pos << "\n"
//             << "Rotation Axis " << rotation_axis[0] << rotation_axis[1] << rotation_axis[2] << endl;
    }

private:
    RoboMath::Vector3 pos;       //Global position
    RoboMath::Vector3 pos_local;   //Local position
    RoboMath::Vector3 rotation_axis_v;
    scalar theta;
    scalar theta_added;
};

class Robo
{
public:
    Robo()
    {
        number_of_elements = 0;
        jacobian = nullptr;
        delta_thetas = nullptr;
        configuration_matrix = nullptr;
        translation_matrix = nullptr;
    };
    Robo(int number_of_elements);
    Robo(initializer_list<Elem> &robo_list);

    void RoboCout();
    void CalculateJacobian();
    void CalculateDeltaThetas();
    void CalculateCurrentPosition();
    void CalculateDeltaS(RoboMath::Vector3 target);
    void AddDeltaThetas();
    void ComputeJointsPosition();
    void UpdateJointsLocalPosition();
    void DeltaThetasCout();
    void SetConfigurationMatrix(const RoboMath::Matrix &A);
    void UpdateTranslationMatrix();
    RoboMath::Vector3 GetEffectorPosition();

private:
    Elem *robot;
    RoboMath::Matrix *jacobian;
    RoboMath::Matrix *configuration_matrix;
    RoboMath::Matrix *translation_matrix;
    RoboMath::Vector *delta_thetas;
    RoboMath::Vector *delta_s;
    int number_of_elements;
    void AllocateForNewRobot();
};



#endif // ROBO_H
