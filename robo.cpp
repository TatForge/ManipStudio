#include "robo.h"

#define CALC_INVERSE_MATRIX

using namespace RoboMath;
const Matrix4x4 q ({0, -1, 0, 0,
                    1,  0, 0, 0,
                    0,  0, 0, 0,
                    0,  0, 0, 0});
const Matrix4x4 q_sq ({-1,   0, 0, 0,
                        0,  -1, 0, 0,
                        0,   0, 0, 0,
                        0,   0, 0, 0});

Vector gv{0, 0, -g, 0};




Robo::Robo()
{
    number_of_elements = 0;
    jacobian = nullptr;
    delta_thetas = nullptr;
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

StateVector Robo::SolveDirectKinematic(const Angles &a, Matrix *const state_matrix)
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

void Robo::SolveDirectDynamic()
{
    CalcUMatrix();
    int ctr(0);
    for (counter i = 0; i < 6; i++)
    {
        for (counter j = i; j < 6; j++)
        {
//            U[i*6 + j].MatrixCout("U");
//            m_cm[j].VectorCout("m_cm");
            m_moments.p[i] += - m_mass[j] * gv.MultiplyTranspose(U[i*6 + j].MultiplyTranspose(m_cm[j]));
        }
        ctr++;
    }
    //Расчет D для i-го элемента манипулятора
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = i; k < 6; k++)
        {
            for (counter j = RoboMath::max(i, k); j < 6; j++)
            {
                m_moments.dv[i] += Trace(U[k*6 + j] * m_ji[k].MultiplyTranspose(U[i*6 + j]));
            }
            m_moments.dv[i] *= m_angles.q_pp[i];
        }
    }

    //    for (int i = 5; i < 6; i++)
    //        for (int j = i; j < 6; j++)
    //        {
    //            std::cout << "U " << i << j << std::endl;
    //            U[i*6 + j].MatrixCout("U");
    //        }

    scalar hikm_buff(0);
    Matrix4x4 u_buff;
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = 0; k < 6; k++)
        {
            for (counter m = 0; m < 6; m++)
            {
                hikm_buff = 0;
                for (counter j = i; j < 6; j++)
                {
                    u_buff.MakeIdentity();
                    for (counter ctr = 0; ctr < j; ctr++)
                    {
                        if (ctr == k && ctr == m)
                            u_buff = u_buff * q_sq * T[ctr];
                        else if (ctr == k || ctr == m)
                            u_buff = u_buff * q * T[ctr];
                        else
                            u_buff = u_buff * T[ctr];
                    }
                    hikm_buff += Trace(u_buff * m_ji[i].MultiplyTranspose(U[i*6 + j])) * m_angles.q_p[i] * m_angles.q_p[i];
                }
                m_moments.hikm[i] += hikm_buff;
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
}

void Robo::LoadSettingsFromFile()
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
}

void Robo::UpdateTMatrix()
{
    RoboMath::Matrix &A = *state_matrix;
    for (int i = 0; i < number_of_elements; i++)
    {
        A(i, 3) += delta_thetas->at(i);
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
        rm::StateVector v = SolveDirectKinematic(a, state_matrix);
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

void Robo::LoadManip()
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
//            m_ji[i].MatrixCout("m_ji");
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
                    U[k*6 + j] = U[k*6 + j] * T[ctr] * q;
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
    jacobian     = new Matrix(EFFECTOR_DEGREE_OF_FREEDOM, number_of_elements);
    state_matrix = new Matrix(number_of_elements, 4);
    delta_thetas = new Vector(number_of_elements);
    theta_vector = new Vector(number_of_elements);
    if (number_of_elements > 0)
        robot = new Elem [number_of_elements];
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


