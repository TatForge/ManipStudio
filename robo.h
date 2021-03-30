#ifndef ROBO_H
#define ROBO_H

#include <iostream>
#include <QSettings>
#include "my_math.h"




using namespace std;
namespace rm = RoboMath;
class Elem
{
public :
    Elem();
    Elem(const rm::Vector3 &pos, const rm::Vector3 &rot, const scalar &theta);
    scalar GetTheta() const {return theta + theta_added;};
    void AddDeltaTheta(const scalar &theta_add);
    inline void SetPositionLocal(const scalar &x, const scalar &y, const scalar &z);
    inline void SetPositionLocal(const rm::Vector3 &pos);
    inline void SetRotationAxisLocal(const rm::Vector3 &rot);
    inline void SetRotationAxis(const rm::Vector3 &rot);
    inline void SetPosition(const rm::Vector3 &V);
//    inline void SetRotation(const rm::Quaternion &rot);
    const rm::Vector3 &GetRotationAxis() const;
    const rm::Vector3 &GetRotationAxisLocal() const;
    const rm::Vector3 &GetPositionLocal() const;
    const rm::Vector3 &GetPosition() const;
//    const rm::Quaternion &GetRotation() const;
//    const rm::Quaternion &GetRotationLocal() const;
protected:
    rm::Vector3 pos;
    rm::Vector3 pos_local;
    rm::Vector3 rotation_axis;
    rm::Vector3 rotation_axis_loc;
//    rm::Quaternion rotation_quat;
//    rm::Quaternion rotation_quat_base;//Положение эффектора, задаваемое на старте программы. Не менять.
    scalar theta;
    scalar theta_added;
};

class Effector : public Elem
{
public:
    inline void SetRotation(const rm::Quaternion &rot);
    const rm::Quaternion &GetRotation() const;
    const rm::Quaternion &GetRotationLocal() const;
private:
    rm::Quaternion rotation_quat;
    rm::Quaternion rotation_quat_base;//Положение эффектора, задаваемое на старте программы. Не менять.
};

class Robo
{
public:
    friend class Emulator;
    friend class Solver;
    friend class Trajectory;
    struct Moments
    {
        scalar tao[JOINTS_COUNT];
        scalar p[JOINTS_COUNT];
        scalar hikm[JOINTS_COUNT];
        scalar dv[JOINTS_COUNT];
        void MakeNull()
        {
            for (uint i = 0; i < JOINTS_COUNT; i++)
            {
                tao[i]  = 0;
                p[i]    = 0;
                hikm[i] = 0;
                dv[i]   = 0;
            }
        }
    };

    Robo();
    Robo(int number_of_elements);
    Robo(initializer_list<Elem> &robo_list);
    ~Robo();
    void RoboCout();
    void CalculateJacobian();
    void CalculateDeltaThetas();
    void CalculateCurrentPosition();
    void CalculateDeltaS(rm::Vector3 target);
    void CalculateDeltaR(rm::Vector3 target_r);
    void AddDeltaThetas();
    void ComputeJointsPosition();
    rm::StateVector SolveForwardKinematic(const rm::Angles &a, rm::Matrix * const state_matrix);
    void SolveInverseKinematic(const scalar &accuracy, const rm::Vector3 &target, const rm::Vector3 &target_r);
    void DeltaThetasCout();
    void SolveForwardDynamic();
    void SolveInverseDynamic();
    void CalcPolynom(const rm::Angles *angle_start, const rm::Angles *angle_stop, const scalar t, RoboMath::Polynom &result);
    void CalcPolynomLine(const rm::Angles *angle_start, const rm::Angles *angle_stop, const scalar t, RoboMath::Polynom &result);
    void CalcPolynomCurve(const rm::Angles *angle_0, const rm::Angles *angle_1, const rm::Angles *angle2, const rm::Angles *angle3);
    void WriteSettingsToFile();
    void LoadSettingsFromFile();
    void UpdateTMatrix();

    bool CheckTrajectoryForLinear(const rm::Polynom pol, rm::Vector3 l1, rm::Vector3 l2, const scalar &t, const scalar &delta_t, const scalar &delta_r);

    bool LoadManip();
    Moments *GetMoments();
    rm::Angles *GetAngles();
    scalar GetTheta(const int &i);
    void SimulateManipPolinoms(RoboMath::Polynom pol, const scalar t);

    rm::Vector3 GetEffectorPosition() const;
    rm::Vector3 GetEffectorRotation() const;
    rm::Quaternion GetEffectorRotationQuat() const;
    rm::Vector3 GetDeltaR() const;
    rm::Vector3 GetDeltaS() const;
    const Elem *GetEffector() const;
    const Elem *GetElem(int number) const;

    //maybe trash

private:
    Elem *robot;

    rm::Matrix *jacobian;
    rm::Matrix4x4 T[JOINTS_COUNT];
    rm::Matrix4x4 T0[JOINTS_COUNT];
    rm::Vector *delta_thetas;
    rm::Vector3 delta_s;
    rm::Vector3 delta_r;
    Effector effector;

    scalar m_mass[JOINTS_COUNT];

    rm::Angles m_angles;
    Moments m_moments;

    rm::Matrix4x4 U[36];
    array<rm::Vector, JOINTS_COUNT> m_cm;
    array<rm::Tensor, JOINTS_COUNT> m_i;
    array<rm::Matrix4x4, JOINTS_COUNT> m_ji;

    rm::Matrix *state_matrix;
    rm::Vector *theta_vector;
    QSettings *manip_settings;
    void RoboSettingsInit();
    void CalcUMatrix();
    int number_of_elements;
    float dt;
    void AllocateForNewRobot();
};



#endif // ROBO_H
