#include "robo.h"

#include <QDebug>

using namespace RoboMath;


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
        robo_it->setRotationLocal(robo_it->GetRotationAxis());
        ++robo_it;
        ++robo_list_it;
    }
    effector_rot_base = Quaternion(Vector3(1, 0, 0), ToRad(90));//0.707, 0, 0, 0.707);
}

Robo::~Robo()
{
    delete jacobian;
    delete delta_thetas;
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
             << endl;
    }
}

void Robo::CalculateJacobian()
{
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
//    this->jacobian->MatrixCout();
}

void Robo:: CalculateDeltaThetas()
{
    Matrix jacobian_transposed = *jacobian;
//    jacobian->MatrixCout();
    jacobian_transposed.Transpose();
    Matrix U(EFFECTOR_DEGREE_OF_FREEDOM, EFFECTOR_DEGREE_OF_FREEDOM);
    U =  (*jacobian) * jacobian_transposed;
//    U.MatrixCout();
    U.AddDiagonal(damping_constant_sq);
    Matrix aug_matrix(U.GetRowsCount(), U.GetColsCount() + 1);
    aug_matrix.SetSubmatrix(U, 0, 0);
//    aug_matrix.MatrixCout();
    aug_matrix.SetSubColumn(this->delta_s, aug_matrix.GetColsCount() - 1, 0);
#ifdef POSITION_AND_ROTATION
    aug_matrix.SetSubColumn(this->delta_r, number_of_elements, 3);
#endif
//    aug_matrix.MatrixCout();
    aug_matrix.SolveGauss();
//    aug_matrix.MatrixCout();
    Vector delta_t(EFFECTOR_DEGREE_OF_FREEDOM);
    aug_matrix.GetTheRoots(delta_t);
//    jacobian_transposed.MatrixCout();
    *this->delta_thetas = jacobian_transposed * delta_t;

    scalar max = delta_thetas->MaxAbs();
//    delta_thetas->VectorCout("Delta thetas ");
    if (max > MaxAngleDls)
    {
        *this->delta_thetas *= MaxAngleDls/max;
    }

//    delta_thetas->VectorCout("Delta thetas normalized");

}

void Robo::CalculateCurrentPosition()
{
    for (int i = 0; i < number_of_elements - 1; i++)
    {

    }
}

void Robo::CalculateDeltaS(Vector3 target)
{
    delta_s.x = target.x - this->Effector()->GetPosition().x;
    delta_s.y = target.y - this->Effector()->GetPosition().y;
    delta_s.z = target.z - this->Effector()->GetPosition().z;
}

void Robo::CalculateDeltaR(Vector3 target_r)
{
    Quaternion end_eff_target_ori;
    Quaternion end_eff_world_ori;
    auto tt = this->Effector()->GetRotationAxis();
    end_eff_world_ori = this->GetEffectorRotationQuat();
    end_eff_target_ori.EulerToQuat(target_r);
    end_eff_world_ori.Invert();
    Quaternion deltaQ = end_eff_target_ori * end_eff_world_ori;
    scalar angle = deltaQ.GetAngle();
    long double angle_ld = deltaQ.GetAngle();
    scalar ang_deg = ToDeg(angle);
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
}

void Robo::ComputeJointsPosition()
{
    Elem *joint_iter        = new Elem;
    Elem *joint_iter_parent = new Elem;
    Elem *joint_iter_buff   = new Elem;
    Vector3  pos, pos_local, rot, local_rot;
    effector_rot = effector_rot_base;
    for (counter elem_n = number_of_elements; elem_n >= 1; elem_n--)
    {
        joint_iter = (this->robot + elem_n);
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
        joint_iter_buff->SetPos(pos);
        joint_iter = (this->robot + elem_n);
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
        joint_iter_buff->SetRotation(rot);
    }
//    cout << " x = " << this->Effector()->GetPosition().x
//         << " y = " << this->Effector()->GetPosition().y
//         << " z = " << this->Effector()->GetPosition().z
//         << endl;
}


void Robo::DeltaThetasCout()
{
    for (counter i = 0; i < this->number_of_elements - 1; i++)
    {
        cout << "number " << i << " = " << *(delta_thetas->GetDataPtr() + i) << endl;
    }
}

Vector3 Robo::GetEffectorPosition() const
{
    return this->Effector()->GetPosition();
}

Vector3 Robo::GetEffectorRotation() const
{
    return this->effector_rot.GetEulerAngles();
}

Quaternion Robo::GetEffectorRotationQuat() const
{
    return this->effector_rot;
}

Vector3 Robo::GetDeltaR() const
{
    return this->delta_r;
}

Vector3 Robo::GetDeltaS() const
{
    return this->delta_s;
}

const Elem *Robo::Effector() const
{
    assert(number_of_elements > 0 && robot && robot != nullptr && "Failed assertion in Robo.Effector()");
    return (robot + number_of_elements - 1);
}

void Robo::AllocateForNewRobot()
{
    jacobian     = new Matrix(EFFECTOR_DEGREE_OF_FREEDOM, number_of_elements);
    delta_thetas = new Vector(number_of_elements);
    if (number_of_elements > 0)
        robot = new Elem [number_of_elements];
}

void Elem::SetPos(Vector3 &V)
{
    this->pos = V;
}

void Elem::SetRotation(const Vector3 &rot)
{
    this->rotation_axis = rot;
}

void Elem::SetPositionLocal(const Vector3 &pos)
{
    this->pos_local = pos;
}

void Elem::setRotationLocal(const Vector3 &rot)
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

void Elem::AddDeltaTheta(const scalar &theta_add){
    this->theta_added += theta_add;
}


