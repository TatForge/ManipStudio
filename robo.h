#ifndef ROBO_H
#define ROBO_H

#include <iostream>
#include <qsettings.h>
#include "my_math.h"

#include <array>

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
    const rm::Vector3 &GetRotationAxis() const;
    const rm::Vector3 &GetRotationAxisLocal() const;
    const rm::Vector3 &GetPositionLocal() const;
    const rm::Vector3 &GetPosition() const;
protected:
    rm::Vector3 pos;
    rm::Vector3 pos_local;
    rm::Vector3 rotation_axis;
    rm::Vector3 rotation_axis_loc;
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
    struct Moments
    {
        scalar tao[JOINTS_COUNT];
        scalar p[JOINTS_COUNT];
        scalar hikm[JOINTS_COUNT];
        scalar dv[JOINTS_COUNT];
        void MakeNull();
    };
    friend class Emulator;
    friend class Solver;
    friend class Trajectory;

    Robo();
    Robo(initializer_list<Elem> &robo_list);
    Robo(int number_of_elements);
    ~Robo();

    bool CheckTrajectoryForLinear(const rm::Polynom pol, rm::Vector3 l1, rm::Vector3 l2, const scalar &t, const scalar &delta_t, const scalar &delta_r);
    void CalcPolynomCurve(const rm::Angles *angle_0, const rm::Angles *angle_1, const rm::Angles *angle2, const rm::Angles *angle3);
    void CalcPolynomLine(const rm::Angles *angle_start, const rm::Angles *angle_stop, const scalar t, RoboMath::Polynom &result);
    void CalcPolynom(const rm::Angles *angle_start, const rm::Angles *angle_stop, const scalar t, RoboMath::Polynom &result);
    void SolveInverseKinematic(const scalar &accuracy, const rm::Vector3 &target, const rm::Vector3 &target_r);
    void SimulateManipPolinoms(RoboMath::Polynom pol, const scalar t);
    void CalculateDeltaR(rm::Vector3 target_r);
    void CalculateDeltaS(rm::Vector3 target);
    void CalculateCurrentPosition();
    void ComputeJointsPosition();
    void LoadSettingsFromFile();
    void CalculateDeltaThetas();
    void SolveForwardDynamic();
    void SolveInverseDynamic();
    void WriteSettingsToFile();
    void CalculateJacobian();
    void DeltaThetasCout();
    void AddDeltaThetas();
    void UpdateTMatrix();
    bool LoadManip();
    void RoboCout();
    rm::StateVector SolveForwardKinematic(const rm::Angles &a, rm::Matrix * const state_matrix);
    vector<scalar> GetMaxJointMoments();
    //joint limits (min, max) in radians; loaded from the "joint limits" group
    //of the settings file (degrees there), defaults otherwise
    vector<pair<scalar, scalar>> GetJointLimits() const;
    void SetJointLimit(int joint, scalar min_rad, scalar max_rad);
    //height of the table / floor plane the robot is mounted on ("workspace/floor_z")
    scalar GetFloorHeight() const;
    void SetFloorHeight(scalar z);
    //rigid-body data of one link in its joint frame, used by the Solver's dynamics:
    //inertia about the frame origin as xx, yy, zz, xy, xz, yz (products as the integral of x*y dm)
    struct LinkDynamics
    {
        scalar mass;
        std::array<scalar, 3> com;
        std::array<scalar, 6> inertia;
    };
    //tool load: a point mass in the flange frame (frame 6)
    struct Payload
    {
        scalar mass;
        std::array<scalar, 3> com;
    };
    //settings groups "dynamics", "payload" and "shape"; defaults otherwise
    const std::array<LinkDynamics, JOINTS_COUNT> &GetLinkDynamics() const;
    void SetLinkDynamics(int link, const LinkDynamics &d);
    Payload GetPayload() const;
    void SetPayload(const Payload &p);
    //drum radius of every joint for the collision model (ArmShape), metres
    std::array<scalar, JOINTS_COUNT> GetJointRadii() const;
    void SetJointRadius(int joint, scalar radius);
    scalar GetTheta(const int &i);
    rm::Angles *GetAngles();
    Moments *GetMoments();

    rm::Quaternion GetEffectorRotationQuat() const;
    rm::Vector3 GetEffectorPosition() const;
    rm::Vector3 GetEffectorRotation() const;
    const Elem *GetElem(int number) const;
    const Elem *GetEffector() const;
    rm::Vector3 GetDeltaR() const;
    rm::Vector3 GetDeltaS() const;
private:
    QSettings *manip_settings;
    rm::Matrix *state_matrix;
    rm::Vector *theta_vector;
    rm::Vector *delta_thetas;
    rm::Matrix *jacobian;
    rm::Vector3 delta_s;
    rm::Vector3 delta_r;
    rm::Angles m_angles;
    rm::Matrix4x4 U[36];
    Effector effector;
    Moments m_moments;
    Elem *robot;
    vector<scalar> max_joint_moments;
    vector<pair<scalar, scalar>> m_joint_limits;
    scalar m_floor_z;
    std::array<LinkDynamics, JOINTS_COUNT> m_link_dynamics;
    Payload m_payload;
    std::array<scalar, JOINTS_COUNT> m_joint_radii;
    void SetDefaultLimits();
    void SetDefaultDynamics();
    array<rm::Matrix4x4, JOINTS_COUNT> m_ji;
    array<rm::Vector, JOINTS_COUNT> m_cm;
    array<rm::Tensor, JOINTS_COUNT> m_i;
    rm::Matrix4x4 T0[JOINTS_COUNT];
    rm::Matrix4x4 T[JOINTS_COUNT];
    scalar m_mass[JOINTS_COUNT];
    void AllocateForNewRobot();
    void RoboSettingsInit();
    void CalcUMatrix();

    int number_of_elements;
    float dt;
};



#endif // ROBO_H
