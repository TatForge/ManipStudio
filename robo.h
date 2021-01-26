#ifndef ROBO_H
#define ROBO_H

#include <iostream>
#include <QSettings>

#include "my_math.h"


//#define dt 0.01
#define POSITION_AND_ROTATION
#define CALC_COUT
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
    void SolveInverseKinematic(const scalar &accuracy, const RoboMath::Vector3 &target, const RoboMath::Vector3 &target_r);;
    void DeltaThetasCout();
    void SolveDirectDynamic();
    void SolveInverseDynamic();
    void CalcPolynomLine(const Angles *angle_start, const Angles *angle_stop, const scalar t, RoboMath::Polynom *result);
    void CalcPolynomCurve(const Angles *angle_0, const Angles *angle_1, const Angles *angle2, const Angles *angle3);
    void WriteSettingsToFile()
    {
        manip_settings = new QSettings("ManipulatorSettigns.ini", QSettings::IniFormat);
        manip_settings->setValue("denavita-hartenberg", 1);
        manip_settings->beginWriteArray("denavita-hartenberg");
        for (int i = 0; i < 6; i++)
        {
            manip_settings->setArrayIndex(i);
            manip_settings->setValue("a", state_matrix->operator()(i, 0));
            manip_settings->setValue("alpha", state_matrix->operator()(i, 1));
            manip_settings->setValue("d", state_matrix->operator()(i, 2));
            manip_settings->setValue("theta", state_matrix->operator()(i, 3));
        }
        manip_settings->endArray();
        manip_settings->setValue("mass array", 1);
        manip_settings->beginWriteArray("mass array");
        for (int i = 0; i < 6; i++)
        {
            manip_settings->setArrayIndex(i);
            manip_settings->setValue(QString::number(i), m_mass[i]);;
        }
        manip_settings->endArray();
        manip_settings->setValue("center of mass", 1);
        manip_settings->beginWriteArray("centers of mass");
        for (int i = 0; i < 6; i++)
        {
            manip_settings->setArrayIndex(i);
            manip_settings->setValue("0", m_cm[i].at(0));
            manip_settings->setValue("1", m_cm[i].at(1));
            manip_settings->setValue("2", m_cm[i].at(2));
            manip_settings->setValue("3", m_cm[i].at(3));
        }
        manip_settings->endArray();
        manip_settings->setValue("inertions", 1);
        manip_settings->beginWriteArray("inertions");
        for (int i = 0; i < 6; i++)
        {
            manip_settings->setArrayIndex(i);
            manip_settings->setValue("0", m_i[i].xx());
            manip_settings->setValue("1", m_i[i].yy());
            manip_settings->setValue("2", m_i[i].zz());
            manip_settings->setValue("3", m_i[i].xy());
            manip_settings->setValue("4", m_i[i].xz());
            manip_settings->setValue("5", m_i[i].yz());
        }
        manip_settings->endArray();
        manip_settings->sync();
    };
    void LoadSettingsFromFile()
    {
        manip_settings = new QSettings("ManipulatorSettigns.ini", QSettings::IniFormat);
        if (!manip_settings->contains("denavita-hartenberg"))
            throw std::string("no denavita-hartenberg array in settings file");
        if (!manip_settings->contains("mass array"))
            throw std::string("no mass array in settings file");
        if (!manip_settings->contains("center of mass"))
            throw std::string("no center of mass array in settings file");
        if (!manip_settings->contains("inertions"))
            throw std::string("no inertions array in settings file");
//        manip_settings->value("joints count", 6);
        int size = manip_settings->beginReadArray("denavita-hartenberg");
        for (int i = 0; i < size; i++)
        {
            manip_settings->setArrayIndex(i);
            state_matrix->operator()(i, 0) = manip_settings->value("a").toDouble();
            state_matrix->operator()(i, 1) = manip_settings->value("alpha").toDouble();
            state_matrix->operator()(i, 2) = manip_settings->value("d").toDouble();
            state_matrix->operator()(i, 3) = manip_settings->value("theta").toDouble();
        }
        manip_settings->endArray();
        manip_settings->beginReadArray("mass array");
        for (int i = 0; i < size; i++)
        {
            manip_settings->setArrayIndex(i);
            m_mass[i] = manip_settings->value("mass").toDouble();
        }
        manip_settings->endArray();
        manip_settings->beginReadArray("centers of mass");
        for (int i = 0; i < size; i++)
        {
            manip_settings->setArrayIndex(i);
            m_cm[i] = RoboMath::Vector({manip_settings->value("0").toDouble(),
                              manip_settings->value("1").toDouble(),
                              manip_settings->value("2").toDouble(),
                              manip_settings->value("3").toDouble()});
        }
        manip_settings->endArray();
        manip_settings->beginReadArray("inertions");
        for (int i = 0; i < size; i++)
        {
            manip_settings->setArrayIndex(i);
            m_i[i] = RoboMath::Tensor({manip_settings->value("0").toDouble(),
                              manip_settings->value("1").toDouble(),
                              manip_settings->value("2").toDouble(),
                              manip_settings->value("3").toDouble(),
                              manip_settings->value("4").toDouble(),
                              manip_settings->value("5").toDouble()});
        }
        manip_settings->endArray();
    };
    void UpdateTMatrix();

    void LoadManip()
    {
        try {
            LoadSettingsFromFile();
            dt = 0.01;
            for (int i = 0; i < number_of_elements; i++)
            {
                m_moments.hikm[i] = 0;
                m_moments.dv[i] = 0;
                m_moments.p[i] = 0;
                m_angles.q_p[i] = 1;
                m_angles.q_pp[i] = 1;
            }
            for (int i = 0; i < number_of_elements; i++)
            {
                m_ji[i] = {{(-m_i[i].xx() + m_i[i].yy() + m_i[i].zz())/2, m_i[i].xy(), m_i[i].xz(), m_mass[i] * m_cm[i](0)},
                         {m_i[i].xy(), (m_i[i].xx() - m_i[i].yy() + m_i[i].zz())/2,  m_i[i].yz(), m_mass[i] * m_cm[i](1)},
                         {m_i[i].xz(), m_i[i].yz(), (m_i[i].xx() + m_i[i].yy() - m_i[i].zz())/2,  m_mass[i] * m_cm[i](2)},
                         {m_mass[i] * m_cm[i](0),  m_mass[i] * m_cm[i](1), m_mass[i] * m_cm[i](2), m_mass[i]}};
                m_ji[i].MatrixCout("m_ji");
            }
            void UpdateTMatrix(RoboMath::Vector *delta_thetas);

//            Elem *robo_it = robot;
//            auto robo_list_it = robo_list.begin();
//            for (counter i = 0; i < number_of_elements; i++)
//            {
//                *robo_it = *robo_list_it;
//                if (i > 0)
//                {
//                    robo_it->SetPositionLocal(Vector3({robo_it->GetPosition().x - (robo_it - 1)->GetPosition().x,
//                                                  robo_it->GetPosition().y - (robo_it - 1)->GetPosition().y,
//                                                  robo_it->GetPosition().z - (robo_it - 1)->GetPosition().z}));
//                }
//                robo_it->SetRotationAxisLocal(robo_it->GetRotationAxis());
//                ++robo_it;
//                ++robo_list_it;
//            }
//            effector.SetRotation(Quaternion(Vector3(1, 0, 0), ToRad(90)));
//            (robot + number_of_elements - 1)->GetRotationAxisLocal();
//            effector.SetRotationAxis((robot + number_of_elements - 1)->GetRotationAxisLocal());
//            effector.SetRotationAxisLocal((robot + number_of_elements - 1)->GetRotationAxisLocal());
//            effector.SetPosition((robot + number_of_elements - 1)->GetPosition());
//            effector.SetPositionLocal(Vector3(0, 0, 0));

        }
        catch (string str)
        {
            cout << str << endl;
        }
    };
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
    RoboMath::Matrix4x4 T0[JOINTS_COUNT];
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
    QSettings *manip_settings;
    void RoboSettingsInit();
    void CalcUMatrix();
    int number_of_elements;
    float dt;
    void AllocateForNewRobot();
};



#endif // ROBO_H
