#include "robo.h"
#include "armshape.h"

#include <stdexcept>
#include <fstream>

#define CALC_INVERSE_MATRIX

using namespace RoboMath;

Vector gv{0, 0, -g, 0};

Robo::Robo()
{
    number_of_elements = 0;
    manip_settings = nullptr;
    state_matrix = nullptr;
    theta_vector = nullptr;
    jacobian = nullptr;
    delta_thetas = nullptr;
    robot = nullptr;
    SetDefaultLimits();
}

Robo::Robo(int number_of_elements)
{
    this->number_of_elements = number_of_elements;
    AllocateForNewRobot();
}

Robo::Robo(initializer_list<Elem> &robo_list)
{
    number_of_elements = robo_list.size();
    AllocateForNewRobot();
    Elem *robo_it = robot;
    auto robo_list_it = robo_list.begin();
    for (counter i = 0; i < robo_list.size(); i++)
    {
        *robo_it = *robo_list_it;
        if (i > 0)
        {
            robo_it->SetPositionLocal(Vector3({robo_it->GetPosition().x - (robo_it - 1)->GetPosition().x,
                                          robo_it->GetPosition().y - (robo_it - 1)->GetPosition().y,
                                          robo_it->GetPosition().z - (robo_it - 1)->GetPosition().z}));
        }
        robo_it->SetRotationAxisLocal(robo_it->GetRotationAxis());
        ++robo_it;
        ++robo_list_it;
    }
    effector.SetRotation(Quaternion(Vector3(1, 0, 0), ToRad(90)));
    (robot + number_of_elements - 1)->GetRotationAxisLocal();
    effector.SetRotationAxis((robot + number_of_elements - 1)->GetRotationAxisLocal());
    effector.SetRotationAxisLocal((robot + number_of_elements - 1)->GetRotationAxisLocal());
    effector.SetPosition((robot + number_of_elements - 1)->GetPosition());
    effector.SetPositionLocal(Vector3(0, 0, 0));
    RoboSettingsInit();
}

Robo::~Robo()
{
    delete jacobian;
    delete delta_thetas;
    delete theta_vector;
    delete state_matrix;
    delete manip_settings;
    delete[] robot;
}

void Robo::RoboCout()
{
    for (counter i = 0; i < number_of_elements; i++)
    {
        cout << "Elem number " << i << endl;
        cout << "x "  << (robot + i)->GetPosition().x
             << " y " << (robot + i)->GetPosition().y
             << " z " << (robot + i)->GetPosition().z
             << "\nRotation axis\n"
             << "x " << (robot + i)->GetRotationAxis().x
             << " y " << (robot + i)->GetRotationAxis().y
             << " z " << (robot + i)->GetRotationAxis().z
             << " Tet " << (robot + i)->GetTheta()
             << endl;
    }
}

void Robo::CalculateJacobian()
{
#ifndef CALC_INVERSE_MATRIX
    Vector3 v_eff = (robot + number_of_elements - 1)->GetPosition();
    Elem *current_el = robot + number_of_elements - 1;
    for (counter i = number_of_elements; i > 0; i--)
    {
        this->jacobian->SetSubColumn(current_el->GetRotationAxis() * (v_eff - current_el->GetPosition()), i - 1, 0);
#ifdef POSITION_AND_ROTATION
        this->jacobian->SetSubColumn(current_el->GetRotationAxis(), i - 1, 3);
#endif
        --current_el;
    }
#ifdef CALC_COUT
    this->jacobian->MatrixCout();
    jacobian->MatrixCout("Jacobian");
#endif
#else
    for (int i = 0; i < number_of_elements; i++)
    {
#ifdef CALC_COUT
        T[i].MatrixCout("T");
        T0[i].MatrixCout("T0");  
#endif
        jacobian->SetSubColumn(Vector3(T0[i](0,2), T0[i](1,2), T0[i](2,2)) * (Vector3({T0[5](0,3), T0[5](1,3), T0[5](2,3)}) - Vector3({T0[i](0,3), T0[i](1,3), T0[i](2,3)})), i, 0);
#ifdef POSITION_AND_ROTATION
        this->jacobian->SetSubColumn(Vector3(T0[i](0,2), T0[i](1,2), T0[i](2,2)), i, 3);
#endif
    }
#ifdef CALC_COUT
    jacobian->MatrixCout("Jacobian");
#endif
#endif
}

void Robo:: CalculateDeltaThetas()
{
    Matrix jacobian_transposed = *jacobian;
#ifdef CALC_COUT
    jacobian->MatrixCout();
#endif
    jacobian_transposed.Transpose();
    Matrix U(6, 6);//EFFECTOR_DEGREE_OF_FREEDOM, EFFECTOR_DEGREE_OF_FREEDOM);
    U = (*jacobian) * jacobian_transposed;
#ifdef CALC_COUT
    U.MatrixCout();
#endif
    U.AddDiagonal(damping_constant_sq);
    Matrix aug_matrix(U.GetRowsCount(), U.GetColsCount() + 1);
    aug_matrix.SetSubmatrix(U, 0, 0);
#ifdef CALC_COUT
    aug_matrix.MatrixCout();
#endif
    aug_matrix.SetSubColumn(this->delta_s, aug_matrix.GetColsCount() - 1, 0);
#ifdef POSITION_AND_ROTATION
    aug_matrix.SetSubColumn(this->delta_r, aug_matrix.GetColsCount() - 1, 3);
#endif
    aug_matrix.SolveGauss();
#ifdef CALC_COUT
    aug_matrix.MatrixCout();
#endif
    Vector delta_t(EFFECTOR_DEGREE_OF_FREEDOM);
    aug_matrix.GetTheRoots(delta_t);
#ifdef CALC_COUT
    delta_t.VectorCout("delta_t");
    aug_matrix.MatrixCout();
    jacobian_transposed.MatrixCout();
#endif
    *this->delta_thetas = jacobian_transposed * delta_t;

    scalar max = delta_thetas->MaxAbs();
#ifdef CALC_COUT
    delta_thetas->VectorCout("Delta thetas ");
#endif
    if (max > MaxAngleDls)
    {
        *this->delta_thetas *= MaxAngleDls/max;
    }
#ifdef CALC_COUT
    delta_thetas->VectorCout("Delta thetas normalized");
#endif

}

void Robo::CalculateCurrentPosition()
{
    for (int i = 0; i < number_of_elements - 1; i++)
    {

    }
}

void Robo::CalculateDeltaS(Vector3 target)
{
#ifndef CALC_INVERSE_MATRIX
    delta_s.x = target.x - this->GetEffector()->GetPosition().x;
    delta_s.y = target.y - this->GetEffector()->GetPosition().y;
    delta_s.z = target.z - this->GetEffector()->GetPosition().z;
#else
    delta_s.x = target.x - T0[5](0,3);
    delta_s.y = target.y - T0[5](1,3);
    delta_s.z = target.z - T0[5](2,3);
#endif
}

void Robo::CalculateDeltaR(Vector3 target_r)
{

    Quaternion end_eff_target_ori;
    Quaternion end_eff_world_ori;
    end_eff_world_ori = T0[5].ConvertToQuaternion();
    end_eff_target_ori.EulerToQuat(target_r);
    end_eff_world_ori.Invert();
    Quaternion deltaQ = end_eff_target_ori * end_eff_world_ori;
    scalar angle = deltaQ.GetAngle();
    Vector3 axis = deltaQ.GetAxis();
    if (angle > M_PI)
        angle -= 2.0 * M_PI;
    else if (angle < - M_PI)
        angle += 2.0 * M_PI;
    Vector3 ang_vel = axis.Normalize() * angle;
    delta_r = ang_vel * 1;
}


void Robo::AddDeltaThetas()
{
    for (int i = 0; i < number_of_elements; i++)
    {
        (robot + i)->AddDeltaTheta(delta_thetas->operator()(i));
    }
    this->RoboCout();
#ifdef CALC_COUT
    this->RoboCout();
#endif
}

void Robo::ComputeJointsPosition()
{
#ifndef CALC_INVERSE_MATRIX
    //Расчет положения джоинтов и эффектора (прямая задача кинематики)
    Elem *joint_iter;
    Elem *joint_iter_parent;
    Elem *joint_iter_buff;
    Vector3  pos, pos_local, rot, local_rot;
    Quaternion effector_rot = effector.GetRotationLocal();
    for (counter elem_n = number_of_elements; elem_n >= 1; elem_n--)
    {
        elem_n == number_of_elements ? joint_iter = &this->effector : joint_iter = (this->robot + elem_n);
        joint_iter_parent = (this->robot + elem_n - 1);
        joint_iter_buff = joint_iter;
        pos = joint_iter->GetPositionLocal();
        for (counter i = elem_n; i > 0; i--)
        {
            //Вращение осуществлять вокруг базовой оси и на базовые дельты
            Quaternion q(joint_iter_parent->GetRotationAxisLocal(), joint_iter_parent->GetTheta());
            pos = q.RotateVector(pos);
            pos += joint_iter_parent->GetPositionLocal();
            --joint_iter;
            --joint_iter_parent;
        }
        joint_iter_buff->SetPosition(pos);
        elem_n == number_of_elements ? joint_iter = &this->effector : joint_iter = (this->robot + elem_n);
        joint_iter_parent = (this->robot + elem_n - 1);
        joint_iter_buff = joint_iter;
        rot = joint_iter->GetRotationAxisLocal();
        for (counter i = elem_n; i > 0; i --)
        {
            Quaternion q(joint_iter_parent->GetRotationAxisLocal(), joint_iter_parent->GetTheta());
            //вращение кватерниона еффектора
            if (elem_n == number_of_elements)
            {
                effector_rot = q * effector_rot;
            }
            rot = q.RotateVector(rot);
            --joint_iter;
            --joint_iter_parent;
        }
        elem_n == number_of_elements ? effector.SetRotation(effector_rot) : joint_iter_buff->SetRotationAxis(rot);
    }
    cout << " x = " << this->GetEffectorPosition().x
         << " y = " << this->GetEffectorPosition().y
         << " z = " << this->GetEffectorPosition().z
         << " a1 = " << this->GetEffectorRotationQuat().GetEulerAngles().x
         << " a2 = " << this->GetEffectorRotationQuat().GetEulerAngles().y
         << " a3 = " << this->GetEffectorRotationQuat().GetEulerAngles().z
         << endl;
#endif
    UpdateTMatrix();
}

StateVector Robo::SolveForwardKinematic(const Angles &a, Matrix *const state_matrix)
{
    RoboMath::Matrix4x4 T[JOINTS_COUNT];
    Matrix &A = *state_matrix;

    T[0] = {{cos(a.q[0]),         -sin(a.q[0]),           0,       A(0,0)},
            {sin(a.q[0]) * cos(0), cos(a.q[0]) * cos(0), -sin(0), -sin(0) * A(0,2)},
            {sin(a.q[0]) * sin(0), cos(a.q[0]) * sin(0),  cos(0),  cos(0) * A(0,2)},
            {0,                    0,                     0,       1}};
    for (counter i = 1; i < 6; i++)
    {
        T[i] = {{cos(a.q[i]),                -sin(a.q[i]),                0,                A(i-1,0)},
                {sin(a.q[i]) * cos(A(i-1,1)), cos(a.q[i]) * cos(A(i-1,1)), -sin(A(i-1,1)), -sin(A(i-1,1)) * A(i,2)},
                {sin(a.q[i]) * sin(A(i-1,1)), cos(a.q[i]) * sin(A(i-1,1)), cos(A(i-1,1)),   cos(A(i-1,1)) * A(i,2)},
                {0,                           0,                           0,               1}};
    }
    Matrix4x4 L =  T[0] * T[1] * T[2] * T[3] * T[4]* T[5];
    return StateVector(L(0,3), L(1,3), L(2,3), L.ConvertToQuaternion().GetEulerAngles());
//    cout << "effector position \nx= " << L(0,3) << " y = " << L(1,3) << " z = " << L(2,3) << endl;
}

void Robo::SolveInverseKinematic(const scalar &accuracy, const Vector3 &target, const Vector3 &target_r) //calc IK from current position to target position
{
    bool solved = true;
    counter ctr(0);
    CalculateDeltaR(target_r);
    CalculateDeltaS(target);
    while (GetDeltaR().Length() + GetDeltaS().Length() > accuracy)
    {
        //need add calc for state_matrix view type
        CalculateJacobian();//calc Jacobian for current configuration

        CalculateDeltaThetas();

//        AddDeltaThetas();

        ComputeJointsPosition();
        CalculateDeltaR(target_r);
        CalculateDeltaS(target);
        ctr++;
        if (ctr > 100)
        {
            cout << "Уравнение не сошлось !_!_!_!_!_!_!_!_!_!_!_!_!" << endl;
            cout << "Цель не достижима _______________" << endl;
            cout << "Положение эффектора " << endl
                 << " ang1 = " << ToDeg(GetEffectorRotationQuat().GetEulerAngles().x)
                 << " ang2 = " << ToDeg(GetEffectorRotationQuat().GetEulerAngles().y)
                 << " ang3 = " << ToDeg(GetEffectorRotationQuat().GetEulerAngles().z)
                 << " x = " << GetEffectorPosition().x
                 << " y = " << GetEffectorPosition().y
                 << " z = " << GetEffectorPosition().z
                 << endl ;
            cout << " Положение цели \n"
                 << " ang1 = " << ToDeg(target_r.x)
                 << " ang2 = " << ToDeg(target_r.y)
                 << " ang3 = " << ToDeg(target_r.z)
                 << " x = " << target.x
                 << " y = " << target.y
                 << " z = " << target.z
                 << endl;
            solved = false;
            break;
        }
    }
    if (solved)
    {
        cout << "Уравнение сошлось за " << ctr << " решений" << endl;
        cout << "Положение эффектора " << endl
             << " ang1 = " << ToDeg(GetEffectorRotationQuat().GetEulerAngles().x)
             << " ang2 = " << ToDeg(GetEffectorRotationQuat().GetEulerAngles().y)
             << " ang3 = " << ToDeg(GetEffectorRotationQuat().GetEulerAngles().z)
             << " x = " << GetEffectorPosition().x
             << " y = " << GetEffectorPosition().y
             << " z = " << GetEffectorPosition().z
             << endl ;
        cout << "Положение цели \n"
             << " ang1 = " << ToDeg(target_r.x)
             << " ang2 = " << ToDeg(target_r.y)
             << " ang3 = " << ToDeg(target_r.z)
             << " x = " << target.x
             << " y = " << target.y
             << " z = " << target.z
             << endl;
    }
}


void Robo::DeltaThetasCout()
{
    for (counter i = 0; i < this->number_of_elements - 1; i++)
    {
        cout << "number " << i << " = " << *(delta_thetas->GetDataPtr() + i) << endl;
    }
}

vector<scalar> Robo::GetMaxJointMoments()
{
    return max_joint_moments;
}

void Robo::SolveForwardDynamic()
{
    //Lagrange-Euler, same model as Solver::SolveForwardDynamic
    CalcUMatrix();
    m_moments.MakeNull();
    scalar buff(0);
    for (counter i = 0; i < 6; i++)
    {
        for (counter j = i; j < 6; j++)
        {
            m_moments.p[i] += - m_mass[j] * gv.MultiplyTranspose(U[i*6 + j].MultiplyTranspose(m_cm[j]));
        }
    }

    //Расчет D для i-го элемента манипулятора
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = 0; k < 6; k++)
        {
            buff = 0;
            for (counter j = RoboMath::max(i, k); j < 6; j++)
            {
                buff += Trace(U[k*6 + j] * m_ji[j].MultiplyTranspose(U[i*6 + j]));
            }
            m_moments.dv[i] += buff * m_angles.q_pp[k];
        }
    }

    Matrix4x4 u_buff;
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = 0; k < 6; k++)
        {
            for (counter m = 0; m < 6; m++)
            {
                if (m_angles.q_p[k] == 0 || m_angles.q_p[m] == 0)
                    continue;
                buff = 0;
                for (counter j = RoboMath::max(i, RoboMath::max(k, m)); j < 6; j++)
                {
                    u_buff.MakeIdentity();
                    for (counter ctr = 0; ctr <= j; ctr++)
                    {
                        u_buff = u_buff * T[ctr];
                        if (ctr == k)
                            u_buff = u_buff * cq;
                        if (ctr == m)
                            u_buff = u_buff * cq;
                    }
                    buff += Trace(u_buff * m_ji[j].MultiplyTranspose(U[i*6 + j]));
                }
                m_moments.hikm[i] += buff * m_angles.q_p[k] * m_angles.q_p[m];
            }
        }
    }
    for ( int i = 0; i < 6; i++)
        m_moments.tao[i] = m_moments.p[i] + m_moments.dv[i] + m_moments.hikm[i];
}

void Robo::SolveInverseDynamic()
{
    //Calc position, speed and acceleration for the next step (+dt)
    for (int i = 0; i < 6; i++)
    {
        m_angles.q_pp[i] = (m_moments.tao[i] - m_moments.hikm[i] - m_moments.p[i])/m_moments.dv[i];
        m_angles.q_p[i]  = m_angles.q_p[i]   + m_angles.q_pp[i] * dt;
        m_angles.q[i]    = m_angles.q[i]     + m_angles.q_p[i] * dt;
    }
}

void Robo::CalcPolynom(const Angles *angle_start, const Angles *angle_stop, const scalar t, Polynom &result)
{
    const auto a0 = angle_start;
    const auto a1 = angle_stop;
    //подстановка вместо t0 нуля и получение аналитических зависимостей позволяет сэкономить кучу процессорного времени на расчет обратной матрицы O(n^3)
    for (int i = 0; i < 6; i++)
    {
        result.joint[i].a[0] =   a0->q[i];
        result.joint[i].a[1] =   a0->q_p[i];
        result.joint[i].a[2] =   0.5 * a0->q_pp[i];
        result.joint[i].a[3] = - 10/pow(t, 3) * a0->q[i] - 6/pow(t, 2) * a0->q_p[i] - 3/(2*t)          * a0->q_pp[i]
                               + 10/pow(t, 3) * a1->q[i] - 4/pow(t, 2) * a1->q_p[i] + 1/(2*t)          * a1->q_pp[i];
        result.joint[i].a[4] =   15/pow(t, 4) * a0->q[i] + 8/pow(t, 3) * a0->q_p[i] + 3/(2*pow(t, 2))  * a0->q_pp[i]
                               - 15/pow(t, 4) * a1->q[i] + 7/pow(t, 3) * a1->q_p[i] - 1/pow(t, 2)      * a1->q_pp[i];
        result.joint[i].a[5] = - 6 /pow(t, 5) * a0->q[i] - 3/pow(t, 4) * a0->q_p[i] - 1/(2*pow(t, 3))  * a0->q_pp[i]
                               + 6 /pow(t, 5) * a1->q[i] - 3/pow(t, 4) * a1->q_p[i] + 1/(2*pow(t, 3))  * a1->q_pp[i];
    }
}

void Robo::CalcPolynomLine(const rm::Angles *angle_start, const rm::Angles *angle_stop, const scalar t, Polynom &result)
{
    // ввести ограничение на минимальное время движения, исходя из максимальной заданной скорости и бросать исключение в случае превышения
    const auto a0 = angle_start;
    const auto a1 = angle_stop;
    //подстановка вместо t0 нуля и получение аналитических зависимостей позволяет сэкономить кучу процессорного времени на расчет обратной матрицы O(n^3)
    for (int i = 0; i < 6; i++)
    {
        result.joint[i].a[0] =   a0->q[i];
        result.joint[i].a[1] =   a0->q_p[i];
        result.joint[i].a[2] =   0.5 * a0->q_pp[i];
        result.joint[i].a[3] = - 10/pow(t, 3) * a0->q[i] - 6/pow(t, 2) * a0->q_p[i] - 3/(2*t)          * a0->q_pp[i]
                               + 10/pow(t, 3) * a1->q[i] - 4/pow(t, 2) * a1->q_p[i] + 1/(2*t)          * a1->q_pp[i];
        result.joint[i].a[4] =   15/pow(t, 4) * a0->q[i] + 8/pow(t, 3) * a0->q_p[i] + 3/(2*pow(t, 2))  * a0->q_pp[i]
                               - 15/pow(t, 4) * a1->q[i] + 7/pow(t, 3) * a1->q_p[i] - 1/pow(t, 2)      * a1->q_pp[i];
        result.joint[i].a[5] = - 6 /pow(t, 5) * a0->q[i] - 3/pow(t, 4) * a0->q_p[i] - 1/(2*pow(t, 3))  * a0->q_pp[i]
                               + 6 /pow(t, 5) * a1->q[i] - 3/pow(t, 4) * a1->q_p[i] + 1/(2*pow(t, 3))  * a1->q_pp[i];
    }
    
}

//void Robo::CalcPolynomCurve(const rm::Angles *angle0, const rm::Angles *angle1, const rm::Angles *angle2, const rm::Angles *angle3)
//{
//    for (int i = 0; i < 6; i++)
//    {
//        scalar a4 = angle1->q[i];
//        scalar a3 = angle1->q[i];
//        scalar a2 = 2*angle0->q_pp[i] + angle1->q[i];
//        scalar a1 = angle0->q_p[i] + angle1->q[i];
//        scalar a0 = angle1->q[i] + angle1->q[i];
//        scalar b3 = angle2->q[i];
//        scalar b2 = angle2->q[i];
//        scalar b1 = angle2->q[i];
//        scalar b0 = angle1->q[i] + angle2->q[i];
//        scalar c4 = angle3->q[i] + 4 * angle3->q_p[i] + 12 * angle3->q_pp[i];
//        scalar c3 = angle3->q[i] + 3 * angle3->q_p[i] + 6 * angle3->q_pp[i];
//        scalar c2 = angle3->q[i] + 2 * angle3->q_p[i] + 2 * angle3->q_pp[i];
//        scalar c1 = angle3->q[i] + angle3->q_p[i];
//        scalar c0 = angle2->q[i] + angle3->q[i];
//    }
//}

void Robo::WriteSettingsToFile()
{
    delete manip_settings;
    manip_settings = new QSettings("ManipulatorSettigns.ini", QSettings::IniFormat);
    manip_settings->setValue("denavita-hartenberg", 1);
    manip_settings->beginWriteArray("denavita-hartenberg");
    for (counter i = 0; i < 6; i++)
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
    for (counter i = 0; i < 6; i++)
    {
        manip_settings->setArrayIndex(i);
        manip_settings->setValue(QString::number(i), m_mass[i]);;
    }
    manip_settings->endArray();
    manip_settings->setValue("center of mass", 1);
    manip_settings->beginWriteArray("centers of mass");
    for (counter i = 0; i < 6; i++)
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
    for (counter i = 0; i < 6; i++)
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
    manip_settings->setValue("moments", 1);
    manip_settings->beginGroup("max moments");
    for (counter i = 0; i < 6; i++)
    {
        manip_settings->setValue(QString::number(i), max_joint_moments.at(i));
    }
    manip_settings->endGroup();
    manip_settings->beginGroup("joint limits");
    for (counter i = 0; i < JOINTS_COUNT; i++)
    {
        manip_settings->setValue(QString::number(i) + "/min_deg", ToDeg(m_joint_limits.at(i).first));
        manip_settings->setValue(QString::number(i) + "/max_deg", ToDeg(m_joint_limits.at(i).second));
    }
    manip_settings->endGroup();
    manip_settings->setValue("workspace/floor_z", m_floor_z);
    static const char *const com_keys[3] = {"com_x", "com_y", "com_z"};
    static const char *const inertia_keys[6] = {"ixx", "iyy", "izz", "ixy", "ixz", "iyz"};
    manip_settings->beginGroup("dynamics");
    for (counter i = 0; i < JOINTS_COUNT; i++)
    {
        const QString link = QString::number(i) + "/";
        manip_settings->setValue(link + "mass", m_link_dynamics[i].mass);
        for (int k = 0; k < 3; k++)
            manip_settings->setValue(link + com_keys[k], m_link_dynamics[i].com[k]);
        for (int k = 0; k < 6; k++)
            manip_settings->setValue(link + inertia_keys[k], m_link_dynamics[i].inertia[k]);
    }
    manip_settings->endGroup();
    manip_settings->beginGroup("payload");
    manip_settings->setValue("mass", m_payload.mass);
    for (int k = 0; k < 3; k++)
        manip_settings->setValue(com_keys[k], m_payload.com[k]);
    manip_settings->endGroup();
    manip_settings->beginGroup("shape");
    for (counter i = 0; i < JOINTS_COUNT; i++)
        manip_settings->setValue(QString::number(i) + "/joint_radius", m_joint_radii[i]);
    manip_settings->endGroup();
    manip_settings->sync();
}

void Robo::LoadSettingsFromFile()
{
    std::ifstream settings_file;
    settings_file.open("ManipulatorSettigns.ini");
    if (!settings_file.is_open())
        throw runtime_error("can't find settings file");
    settings_file.close();
    delete manip_settings;
    manip_settings = new QSettings("ManipulatorSettigns.ini", QSettings::IniFormat);
    if (!manip_settings->contains("denavita-hartenberg"))
        throw runtime_error("no denavita-hartenberg data in settings file");
    if (!manip_settings->contains("mass array"))
        throw runtime_error("no mass data in settings file");
    if (!manip_settings->contains("center of mass"))
        throw runtime_error("no center of mass data in settings file");
    if (!manip_settings->contains("inertions"))
        throw runtime_error("no inertions data in settings file");
    if (!manip_settings->contains("moments"))
        throw runtime_error("no max moments data in settings file");
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
        m_mass[i] = manip_settings->value(QString::number(i)).toDouble();
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
    manip_settings->beginGroup("max moments");
    for (counter i = 0; i < 6; i++)
    {
        max_joint_moments.at(i) = manip_settings->value(QString::number(i)).toDouble();
    }
    manip_settings->endGroup();
    //optional: joint limits (degrees) and floor height; defaults stay for missing keys
    manip_settings->beginGroup("joint limits");
    for (counter i = 0; i < JOINTS_COUNT; i++)
    {
        const QString min_key = QString::number(i) + "/min_deg";
        const QString max_key = QString::number(i) + "/max_deg";
        if (manip_settings->contains(min_key))
            m_joint_limits.at(i).first = ToRad(manip_settings->value(min_key).toDouble());
        if (manip_settings->contains(max_key))
            m_joint_limits.at(i).second = ToRad(manip_settings->value(max_key).toDouble());
    }
    manip_settings->endGroup();
    if (manip_settings->contains("workspace/floor_z"))
        m_floor_z = manip_settings->value("workspace/floor_z").toDouble();
    //optional: dynamics, payload, collision shape; defaults stay for missing keys
    auto read = [this](const QString &key, scalar &value) {
        if (manip_settings->contains(key))
            value = manip_settings->value(key).toDouble();
    };
    static const char *const com_keys[3] = {"com_x", "com_y", "com_z"};
    static const char *const inertia_keys[6] = {"ixx", "iyy", "izz", "ixy", "ixz", "iyz"};
    for (counter i = 0; i < JOINTS_COUNT; i++)
    {
        LinkDynamics d = m_link_dynamics[i];
        const QString link = "dynamics/" + QString::number(i) + "/";
        read(link + "mass", d.mass);
        for (int k = 0; k < 3; k++)
            read(link + com_keys[k], d.com[k]);
        for (int k = 0; k < 6; k++)
            read(link + inertia_keys[k], d.inertia[k]);
        SetLinkDynamics(int(i), d);
        scalar radius = m_joint_radii[i];
        read("shape/" + QString::number(i) + "/joint_radius", radius);
        SetJointRadius(int(i), radius);
    }
    Payload p = m_payload;
    read("payload/mass", p.mass);
    for (int k = 0; k < 3; k++)
        read(QString("payload/") + com_keys[k], p.com[k]);
    SetPayload(p);

}

void Robo::UpdateTMatrix()
{
    RoboMath::Matrix &A = *state_matrix;
    for (int i = 0; i < number_of_elements; i++)
    {
        A(i, 3) += delta_thetas->at(i);
        delta_thetas->at(i) = 0; //each IK step is applied exactly once
    }
//    delta_thetas->VectorCout("delta_theta");
//    A.MatrixCout("A matrix");
    T[0] = {{cos(A(0,3)),         -sin(A(0,3)),           0,       A(0,0)},
            {sin(A(0,3)) * cos(0), cos(A(0,3)) * cos(0), -sin(0), -sin(0) * A(0,2)},
            {sin(A(0,3)) * sin(0), cos(A(0,3)) * sin(0),  cos(0),  cos(0) * A(0,2)},
            {0,                    0,                     0,       1}};
    for (counter i = 1; i < number_of_elements; i++)
    {
        T[i] = {{cos(A(i,3)),                 -sin(A(i,3)),                0,              A(i-1,0)},
                {sin(A(i,3)) * cos(A(i-1,1)), cos(A(i,3)) * cos(A(i-1,1)), -sin(A(i-1,1)), -sin(A(i-1,1)) * A(i,2)},
                {sin(A(i,3)) * sin(A(i-1,1)), cos(A(i,3)) * sin(A(i-1,1)), cos(A(i-1,1)),  cos(A(i-1,1)) * A(i,2)},
                {0,                           0,                           0,              1}};
    }
    T0[0] = T[0];
    T0[1] = T[0] * T[1];
    T0[2] = T[0] * T[1] * T[2];
    T0[3] = T[0] * T[1] * T[2] * T[3];
    T0[4] = T[0] * T[1] * T[2] * T[3] * T[4];
    T0[5] = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
    effector.SetPosition(Vector3(T0[5](0,3), T0[5](1,3), T0[5](2,3)));
    effector.SetRotation(T0[5].ConvertToQuaternion());
}

bool Robo::CheckTrajectoryForLinear(const Polynom pol, Vector3 l1, Vector3 l2, const scalar &t, const scalar &delta_t, const scalar &delta_r)
{
    rm::Angles a;
    scalar cur_t(0);
    while (t - cur_t > 10e-4)
    {
        for (int i = 0; i < number_of_elements; i++)
        {
            //                pol.joint[i].Cout("pol");
            a.q[i] = pol.GetAngle(i, cur_t);
        }
        rm::StateVector v = SolveForwardKinematic(a, state_matrix);
        auto distance = CalcDistancePointToLine(v.GetCoord(), &l1, &l2);
        if (distance > delta_r)
        {
            cout <<" ALARM BAD TRAJECTORY, DISTANCE = " << distance << endl;
            return false;
        } else
        {
            cout << "ALL GOOOOOOOD, DISTANCE = " << distance << endl;
        }
        //            cout << "x = " << v.x << " y = " << v.y << " z = " << v.z
        //                 << "\nroll = " << ToDeg(v.roll) << " pitch = " << ToDeg(v.pitch) << " yaw " << ToDeg(v.yaw) << endl;
        cur_t += delta_t;
    }
    return true;
}

bool Robo::LoadManip()
{
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
//            m_ji[i].MatrixCout("m_ji");
    }
    UpdateTMatrix();
    return true;
}

Robo::Moments *Robo::GetMoments()
{
    return &m_moments;
}

rm::Angles *Robo::GetAngles()
{
    return &m_angles;
}

scalar Robo::GetTheta(const int &i)
{
    return state_matrix->at(i, 3);
}

void Robo::SimulateManipPolinoms(Polynom pol, const scalar t)
{
    scalar q[6];
    for (int i = 0; i < 6; i++)
    {
        q[i] = pol.GetAngle(i, t);
    }
    cout << "manip station " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << endl;
}

//void Robo::CreateStateMatrixes()
//{
//    theta_vector = new Vector({0 * M_PI/180,
//            0 * M_PI/180,
//            0 * M_PI/180,
//            0 * M_PI/180,
//            0 * M_PI/180,
//            0 * M_PI/180});
//    state_matrix = new Matrix({{0,      -M_PI/2, 0.2325, theta_vector->operator()(0)},
//                               {0.375,   0,      0.100,  theta_vector->operator()(1)},
//                               {0.295,   0,     -0.100,  theta_vector->operator()(2)},
//                               {0,      -M_PI/2, 0.1205, theta_vector->operator()(3)},
//                               {0,       M_PI/2, 0.171,  theta_vector->operator()(4)},
//                               {0,       0,      0.1226, theta_vector->operator()(5)}});

//}

Vector3 Robo::GetEffectorPosition() const
{
    return this->GetEffector()->GetPosition();
}

Vector3 Robo::GetEffectorRotation() const
{
#ifndef CALC_INVERSE_MATRIX
    return effector.GetRotation().GetEulerAngles();
#else
    //Доработать с углами, по положению работает, по углам не понятно пока
    state_matrix->MatrixCout("state matrix");
    T0[5].MatrixCout("T0[5]");
    double pitch = atan(T0[5].at(2,0)/T0[5].at(2,1));
    double yaw   = atan(T0[5].at(0,2)/T0[5].at(1,2));
    double roll  = atan((sin(pitch) * T0[5].at(2,0) - cos(pitch) * T0[5].at(2,1))/T0[5].at(2,2));
    return Vector3(yaw, pitch, roll);
#endif
}

Quaternion Robo::GetEffectorRotationQuat() const
{
    return effector.GetRotation();
}

Vector3 Robo::GetDeltaR() const
{
    return this->delta_r;
}

Vector3 Robo::GetDeltaS() const
{
    return this->delta_s;
}

const Elem *Robo::GetEffector() const
{
    //    assert(number_of_elements > 0 && robot && robot != nullptr && "Failed assertion in Robo.Effector()");
    return &effector;
}

const Elem *Robo::GetElem(int number) const
{
    return (robot + number);
}

void Robo::RoboSettingsInit()
{
    dt = 0.01;
    for (int i = 0; i < 6; i++)
    {
        m_moments.hikm[i] = 0;
        m_moments.dv[i] = 0;
        m_moments.p[i] = 0;
        m_angles.q_p[i] = 1;
        m_angles.q_pp[i] = 1;
    }
    m_mass[0] = 0.5982;
    m_mass[1] = 1.1495;
    m_mass[2] = 0.6399;
    m_mass[3] = 0.3551;
    m_mass[4] = 0.1747;
    m_mass[5] = 0.1667;

    *state_matrix = {{0,    -M_PI/2, 0.2325, 0 * M_PI/180},
                    {0.375,  0,      0.100,  0 * M_PI/180},
                    {0.295,  0,     -0.100,  0 * M_PI/180},
                    {0,     -M_PI/2, 0.1205, 0 * M_PI/180},
                    {0,      M_PI/2, 0.171,  0 * M_PI/180},
                    {0,      0,      0.1226, 0 * M_PI/180}};
    for (int i = 0; i < 6; i++)
    {
        m_angles.q[i] = state_matrix->at(i, 3);
    }
    Matrix A {{0,     -M_PI/2, 0.2325, 0 * M_PI/180},                          //(robot + 0)->GetTheta()},
              {0.375,  0,      0.100,  0 * M_PI/180},//(robot + 1)->GetTheta()},
              {0.295,  0,     -0.100,  0 * M_PI/180},//(robot + 2)->GetTheta()},
              {0,     -M_PI/2, 0.1205, 0 * M_PI/180},//(robot + 3)->GetTheta()},
              {0,      M_PI/2, 0.171,  0 * M_PI/180},//(robot + 4)->GetTheta()},
              {0,      0,      0.1226, 0 * M_PI/180}};//(robot + 5)->GetTheta()}};
    m_cm[0] = Vector({0, 0, 0, 1});
    m_cm[1] = Vector({0, 0, 0, 1});
    m_cm[2] = Vector({0, 0, 0, 1});
    m_cm[3] = Vector({0, 0, 0, 1});
    m_cm[4] = Vector({0, 0, 0, 1});
    m_cm[5] = Vector({0, 0, 0, 1});
    m_i[0] = {0.000588, 0.000587, 0.000929, -1.299e-8, 2.44e-8, 1.79e-6};
    m_i[1] = {0.000928, 0.001512, 0.000937, 1.86e-5,  1.55e-10, 5.34e-12};
    m_i[2] = {0.001759, 0.000747, 0.001913, 2.32e-8,  6.37e-9,  1.68e-6};
    m_i[3] = {0.000198, 0.000479, 0.002059, 3.16e-11, 5.59e-10, 1.59e-7};
    m_i[4] = {9.826e-5, 0.000128, 0.000114, -2.74e-9, 4.41e-9,  5.82e-7};
    m_i[5] = {5.529e-5, 5.359e-5, 8.415e-5, 1.99e-10, 2.53e-9,  1.85e-7};

    for (int i = 0; i < 6; i++)
    {
        m_ji[i] = {{(-m_i[i].xx() + m_i[i].yy() + m_i[i].zz())/2, m_i[i].xy(), m_i[i].xz(), m_mass[i] * m_cm[i](0)},
                 {m_i[i].xy(), (m_i[i].xx() - m_i[i].yy() + m_i[i].zz())/2,  m_i[i].yz(), m_mass[i] * m_cm[i](1)},
                 {m_i[i].xz(), m_i[i].yz(), (m_i[i].xx() + m_i[i].yy() - m_i[i].zz())/2,  m_mass[i] * m_cm[i](2)},
                 {m_mass[i] * m_cm[i](0),  m_mass[i] * m_cm[i](1), m_mass[i] * m_cm[i](2), m_mass[i]}};
        m_ji[i].MatrixCout("m_ji");
    }
    T[0] = {{cos(A(0,3)),         -sin(A(0,3)),           0,       A(0,0)},
            {sin(A(0,3)) * cos(0), cos(A(0,3)) * cos(0), -sin(0), -sin(0) * A(0,2)},
            {sin(A(0,3)) * sin(0), cos(A(0,3)) * sin(0),  cos(0),  cos(0) * A(0,2)},
            {0,                    0,                     0,       1}};
    for (counter i = 1; i < 6; i++)
    {
        T[i] = {{cos(A(i,3)),                 -sin(A(i,3)),                0,              A(i-1,0)},
                {sin(A(i,3)) * cos(A(i-1,1)), cos(A(i,3)) * cos(A(i-1,1)), -sin(A(i-1,1)), -sin(A(i-1,1)) * A(i,2)},
                {sin(A(i,3)) * sin(A(i-1,1)), cos(A(i,3)) * sin(A(i-1,1)), cos(A(i-1,1)),  cos(A(i-1,1)) * A(i,2)},
                {0,                           0,                           0,              1}};
    }
}

void Robo::CalcUMatrix()
{
    for (counter k = 0; k < 6; k++)
    {
        for (counter j = k; j < 6; j++)
        {
            U[k*6 + j].MakeIdentity();
            //Считается Ujk для конкретного i,j,k
            for (int ctr = 0; ctr <= j; ctr++)
            {
//                T[ctr].MatrixCout();
                if (ctr == k)
                    U[k*6 + j] = U[k*6 + j] * T[ctr] * cq;
                else
                {
                    U[k*6 + j] = U[k*6 + j] * T[ctr];
                }
//                U[k*6 + j].MatrixCout();
            }
//                cout << "U" << k << j << endl;
//            U[k*6 + j].MatrixCout("U");
        }
    }
}

void Robo::AllocateForNewRobot()
{
    manip_settings = nullptr;
    jacobian     = new Matrix(EFFECTOR_DEGREE_OF_FREEDOM, number_of_elements);
    state_matrix = new Matrix(number_of_elements, 4);
    delta_thetas = new Vector(number_of_elements, 0);
    theta_vector = new Vector(number_of_elements, 0);
    jacobian->MakeZero();
    state_matrix->MakeZero();
    max_joint_moments.resize(6);
    robot = number_of_elements > 0 ? new Elem [number_of_elements] : nullptr;
    SetDefaultLimits();
}

void Robo::SetDefaultLimits()
{
    //degrees: base, shoulder (+30 keeps the elbow above the mounting plane), elbow, wrist x3
    const scalar limits_deg[JOINTS_COUNT][2] = {{-170, 170}, {-150, 30}, {-175, 175}, {-180, 180}, {-135, 135}, {-360, 360}};
    m_joint_limits.resize(JOINTS_COUNT);
    for (int i = 0; i < JOINTS_COUNT; i++)
        m_joint_limits[i] = {ToRad(limits_deg[i][0]), ToRad(limits_deg[i][1])};
    m_floor_z = 0;
    //motor limits, N*m (WriteSettingsToFile used to force these on every save)
    max_joint_moments = {180, 180, 120, 120, 60, 60};
    SetDefaultDynamics();
}

void Robo::SetDefaultDynamics()
{
    //the model the Solver used to hard-code
    m_link_dynamics = {{{2.7600, {{0, -0.0046, -0.0467}}, {{0.0120, 0.0117, 0.0032, 0, 0, 0}}},
                        {2.9325, {{0.3251, 0, -0.0204}}, {{0.0055, 0.4014, 0.3983, 0, 0, -0.0276}}},
                        {2.5773, {{0.2869, 0, 0.0235}}, {{0.0050, 0.2629, 0.2596, 0, 0, 0.0228}}},
                        {1.6183, {{0.0757, 0, 0.0011}}, {{0.0007, 0.0127, 0.0126, 0, 0, 0}}},
                        {1.3935, {{-0.0023, 0, 0.0364}}, {{0.0033, 0.0033, 0.006, 0, 0, 0}}},
                        {0.2498, {{0.0001, 0, 0.0193}}, {{0.0002, 0.0002, 0.0001, 0, 0, 0}}}}};
    //5 kg at the centre of mass of the last link
    m_payload = {5.0, {{0.0001, 0, 0.0193}}};
    m_joint_radii = ArmShape::DefaultJointRadii();
}

Robo::DHParams Robo::GetDH(int joint) const
{
    if (joint < 0 || joint >= JOINTS_COUNT)
        throw std::out_of_range("joint " + std::to_string(joint) + " does not exist");
    return {state_matrix->at(joint, 0), state_matrix->at(joint, 1), state_matrix->at(joint, 2)};
}

void Robo::SetDH(int joint, const DHParams &p)
{
    if (joint < 0 || joint >= JOINTS_COUNT)
        throw std::out_of_range("joint " + std::to_string(joint) + " does not exist");
    if (!std::isfinite(p.a) || !std::isfinite(p.alpha) || !std::isfinite(p.d))
        throw std::invalid_argument("DH parameters must be finite numbers");
    (*state_matrix)(joint, 0) = p.a;
    (*state_matrix)(joint, 1) = p.alpha;
    (*state_matrix)(joint, 2) = p.d;
    UpdateTMatrix();
}

void Robo::SetMaxJointMoment(int joint, scalar value)
{
    if (joint < 0 || joint >= JOINTS_COUNT)
        throw std::out_of_range("joint " + std::to_string(joint) + " does not exist");
    if (!(value > 0))
        throw std::invalid_argument("max joint moment must be positive");
    max_joint_moments.at(joint) = value;
}

const std::array<Robo::LinkDynamics, JOINTS_COUNT> &Robo::GetLinkDynamics() const
{
    return m_link_dynamics;
}

void Robo::SetLinkDynamics(int link, const LinkDynamics &d)
{
    if (link < 0 || link >= JOINTS_COUNT)
        throw std::out_of_range("link " + std::to_string(link) + " does not exist");
    if (d.mass < 0)
        throw std::invalid_argument("link mass must not be negative");
    m_link_dynamics[link] = d;
}

Robo::Payload Robo::GetPayload() const
{
    return m_payload;
}

void Robo::SetPayload(const Payload &p)
{
    if (p.mass < 0)
        throw std::invalid_argument("payload mass must not be negative");
    m_payload = p;
}

std::array<scalar, JOINTS_COUNT> Robo::GetJointRadii() const
{
    return m_joint_radii;
}

void Robo::SetJointRadius(int joint, scalar radius)
{
    if (joint < 0 || joint >= JOINTS_COUNT)
        throw std::out_of_range("joint " + std::to_string(joint) + " does not exist");
    if (!(radius > 0))
        throw std::invalid_argument("joint radius must be positive");
    m_joint_radii[joint] = radius;
}

vector<pair<scalar, scalar>> Robo::GetJointLimits() const
{
    return m_joint_limits;
}

void Robo::SetJointLimit(int joint, scalar min_rad, scalar max_rad)
{
    m_joint_limits.at(joint) = {min_rad, max_rad};
}

scalar Robo::GetFloorHeight() const
{
    return m_floor_z;
}

void Robo::SetFloorHeight(scalar z)
{
    m_floor_z = z;
}

void Elem::SetPosition(const Vector3 &V)
{
    this->pos = V;
}

void Effector::SetRotation(const Quaternion &rot)
{
    this->rotation_quat = rot;
}

void Elem::SetRotationAxis(const Vector3 &rot)
{
    this->rotation_axis = rot;
}

//void Elem::SetRotationAxis(const Quaternion &rot)
//{
//    this->rotation_quat_base = rot;
//}

//void Elem::SetRotationAxisLocal(const Quaternion &rot)
//{
//    this->rotation_quat_loc = rot;
//}

void Elem::SetPositionLocal(const Vector3 &pos)
{
    this->pos_local = pos;
}

void Elem::SetRotationAxisLocal(const Vector3 &rot)
{
    this->rotation_axis_loc = rot;
}

void Elem::SetPositionLocal(const scalar &x, const scalar &y, const scalar &z)
{
    pos_local = RoboMath::Vector({x, y, z});
}

Elem::Elem()
{
    this->pos             = Vector3();
    this->pos_local       = Vector3();
    this->rotation_axis = Vector3();
    this->theta = 0;
    this->theta_added = 0;
}
Elem::Elem(const Vector3 &pos, const Vector3 &rot, const scalar &theta)
{
    this->pos = pos;
    this->pos_local = Vector3();
    this->rotation_axis = rot;
    this->theta = theta;
    this->theta_added = 0;
}

const Vector3 &Elem::GetRotationAxis() const
{
    return this->rotation_axis;
}

const Vector3 &Elem::GetRotationAxisLocal() const
{
    return this->rotation_axis_loc;
}

const Vector3 &Elem::GetPositionLocal() const
{
    return this->pos_local;
}

const Vector3 &Elem::GetPosition() const
{
    return this->pos;
}

const Quaternion &Effector::GetRotation() const
{
    return this->rotation_quat;
}

//const Quaternion &Elem::GetRotationBase() const
//{
//    return this->rotation_quat_base;
//}

const Quaternion &Effector::GetRotationLocal() const
{
    return this->rotation_quat_base;
}

void Elem::AddDeltaTheta(const scalar &theta_add){
    this->theta_added += theta_add;
}



void Robo::Moments::MakeNull()
{
    for (uint i = 0; i < JOINTS_COUNT; i++)
    {
        tao[i]  = 0;
        p[i]    = 0;
        hikm[i] = 0;
        dv[i]   = 0;
    }
}
