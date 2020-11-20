#include "robo.h"

#include <QDebug>

using namespace RoboMath;


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

//void Robo::CalculateJacobian(const Vector3 target)
//{
//    double x, y, z, orx, ory, orz;
//    Elem *effector = robot + number_of_elements - 1;
//    Elem *current_element = robot + number_of_elements - 2;

//    for(int i = number_of_elements - 1; i > 0; i--)
//    {
//        x = current_element->GetPosition().x - target->GetPosition().x;
//        y = current_element->GetPosition().y - target->GetPosition().y;
//        z = current_element->GetPosition().z - target->GetPosition().z;
//        orx = current_element->GetRotationAxis().x;
//        ory = current_element->GetRotationAxis().y;
//        orz = current_element->GetRotationAxis().z;
//        this->jacobian->operator()(0, i - 1) = y * orz - z * ory;
//        this->jacobian->operator()(1, i - 1) = z * orx - x * orz;
//        this->jacobian->operator()(2, i - 1) = x * ory - y * orx;
//        --current_element;
//    }
//    cout << " Jacobian = " << endl;
//    this->jacobian->MatrixCout();
//}

void Robo::CalculateJacobian()
{
    double x, y, z, orx, ory, orz;
    Elem *effector = robot + number_of_elements - 1;
    Elem *current_element = robot + number_of_elements - 2;

    for(counter i = number_of_elements - 1; i > 0; i--)
    {
        x = current_element->GetPosition().x - effector->GetPosition().x;
        y = current_element->GetPosition().y - effector->GetPosition().y;
        z = current_element->GetPosition().z - effector->GetPosition().z;
        orx = current_element->GetRotationAxis().x;
        ory = current_element->GetRotationAxis().y;
        orz = current_element->GetRotationAxis().z;
        this->jacobian->operator()(0, i - 1) = y * orz - z * ory;
        this->jacobian->operator()(1, i - 1) = z * orx - x * orz;
        this->jacobian->operator()(2, i - 1) = x * ory - y * orx;

        --current_element;
    }
    //Для параметров Денавита-Хартенберга
//    for (counter i = number_of_elements - 1; i > 0; i--)
//    {
//        this->jacobian->SetColumn(effector->GetPosition() - current_element->GetPosition(), i);//current_element->GetRotationAxis() * (effector->GetPosition() - current_element->GetPosition());

////        this->jacobian->operator()(1, i) = ;
////        this->jacobian->operator()(2, i) = ;
//    }
//    this->jacobian->MatrixCout();
}

void Robo::CalculateDeltaThetas()
{
    Matrix jacobian_transposed = *jacobian;
    jacobian_transposed.Transpose();
    Matrix U(EFFECTOR_DEGREE_OF_FREEDOM, EFFECTOR_DEGREE_OF_FREEDOM);
    U = *jacobian * jacobian_transposed;
    U.AddDiagonal(damping_constant_sq);
    Matrix aug_matrix(EFFECTOR_DEGREE_OF_FREEDOM, 4);
    aug_matrix.SetSubmatrix(U);
    aug_matrix.SetColumn(*this->delta_s, 4);
    aug_matrix.SolveGauss();
    Vector delta_t(EFFECTOR_DEGREE_OF_FREEDOM);
    aug_matrix.GetTheRoots(delta_t);
    *this->delta_thetas = jacobian_transposed * delta_t;

    scalar max = delta_thetas->MaxAbs();
    if (max > MaxAngleDls)
    {
        *this->delta_thetas *= MaxAngleDls/max;
    }
}

void Robo::CalculateCurrentPosition()
{
    for (int i = 0; i < number_of_elements - 1; i++)
    {

    }
}

void Robo::CalculateDeltaS(Vector3 target)
{
    *(delta_s->GetDataPtr() + 0) = target.x - this->Effector()->GetPosition().x;
    *(delta_s->GetDataPtr() + 1) = target.y - this->Effector()->GetPosition().y;
    *(delta_s->GetDataPtr() + 2) = target.z - this->Effector()->GetPosition().z;
//    cout << " delta_s " << endl << *(delta_s->GetDataPtr() + 0)
//         << " " << *(delta_s->GetDataPtr() + 1) << " "
    //         << *(delta_s->GetDataPtr() + 2) << endl;
}

void Robo::CalculateDeltaR(Vector3 target_r)
{
    Quaternion end_eff_target_ori;
    Quaternion end_eff_world_ori;
    auto tt = this->Effector()->GetRotationAxis();
    end_eff_world_ori.EulerToQuat(this->Effector()->GetRotationAxis());
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
    for (int i = 0; i < number_of_elements - 1; i++)
    {
        (robot + i)->AddDeltaTheta(delta_thetas->operator()(i));
    }
}

void Robo::ComputeJointsPosition()
{
    Elem *joint_iter;
    Elem *joint_iter_parent;

    Elem *joint_iter_buff;
    Vector3  pos, pos_local, rot, local_rot;
    for (counter elem_n = number_of_elements - 1; elem_n >= 1; elem_n--)
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

//            pos_local = joint_iter->GetPositionLocal();
//            pos += joint_iter_parent->GetPositionLocal();//q.RotateVector(pos_local);
//            pos += pos;//joint_iter_parent->GetPositionLocal();
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
            rot = q.RotateVector(rot);
            --joint_iter;
            --joint_iter_parent;
        }
        joint_iter_buff->SetRotation(rot);
//        cout << " Elem number " << elem_n << " " << pos.x << " " << pos.y << " " << pos.z << endl;

//        joint_iter_buff->SetPos(pos);
//        rot = joint_iter->GetRotationAxis();
//        for (counter i = elem_n; i > 0; i --)
//        {
//            Quaternion q(joint_iter_parent->GetRotationAxis(), joint_iter_parent->GetTheta());
//            rot = q.RotateVector(rot);
//        }
//        joint_iter_buff->SetRotation(rot);

//        cout << " Elem number rot" << elem_n << " " << rot.x << " " << rot.y << " " << rot.z << endl;
    }


//    for (counter elem_n = number_of_elements - 1; elem_n >= 1; elem_n--)
//    {
//        joint_iter = (this->robot + elem_n);
//        joint_iter_parent = (this->robot + elem_n - 1);
//        for (counter i = elem_n; i > 0; i--)
//        {
//            Quaternion q(joint_iter_parent->GetRotationAxis(), joint_iter_parent->GetTheta());
//            rot = joint_iter->GetRotationAxis();
//            rot = q.RotateVector(rot);
//            joint_iter->SetRotation(rot);
//            --joint_iter;
//            --joint_iter_parent;
//        }
//    cout << " Elem number rot" << elem_n << " " << rot.x << " " << rot.y << " " << rot.z << endl;
//    }

//    for (int i = 1; i < number_of_elements - 1; i++)
//    {
//        local_pos = joint_iter->GetPosLocal();
//        for (int j = i; j > 0; j++)
//        {
//            local_pos.Rotate(joint_iter_parent->GetTheta(), joint_iter_parent->GetRotation());
//            --joint_iter;
//            --joint_iter_parent;
////            this->robot->SetPos()
//        }

//    }

//    for (int i = 1; i < number_of_elements; i++)
//    {
//        //Расчет глобальной позиции каждого следующего элемента манипулятора
//        local_pos = joint_iter->GetPosLocal();
//        local_pos.Rotate((joint_iter - 1)->GetTheta(), (joint_iter - 1)->GetRotation());
//        pos = (joint_iter - 1)->GetPosition() + local_pos;
//        joint_iter->SetPos(pos);
//        for (int j = i; j > 0; j++)
//        {

//        }

//        //Расчет глобальных осей вращения каждого следующего элемента манипулятора
//        rot = joint_iter->GetRotation();
//        rot.Rotate((joint_iter - 1)->GetTheta(), (joint_iter - 1)->GetRotation());



//        ++joint_iter;
//    }
//    //update Rotation
//    joint_iter = (this->robot + 1);
//    for (int i = 1; i < number_of_elements; i++)
//    {

//    }
}

void Robo::UpdateJointsLocalPosition()
{

//    Vector3 local_pos_buff;
//    for (counter i = 1; i < this->number_of_elements; i++)
//    {
//        local_pos_buff.x = (robot + i)->GetPosition().x - (robot + i - 1)->GetPosition().x;
//        local_pos_buff.y = (robot + i)->GetPosition().y - (robot + i - 1)->GetPosition().y;
//        local_pos_buff.z = (robot + i)->GetPosition().z - (robot + i - 1)->GetPosition().z;
//        (robot + i)->SetLocalPos(local_pos_buff);
//    }
}

void Robo::DeltaThetasCout()
{
    for (counter i = 0; i < this->number_of_elements - 1; i++)
    {
        cout << "number " << i << " = " << *(delta_thetas->GetDataPtr() + i) << endl;
    }
}

void Robo::SetConfigurationMatrix(const Matrix &A)
{
    assert(A.GetColsCount() == this->configuration_matrix->GetColsCount() &&
           A.GetRowsCount() == this->configuration_matrix->GetRowsCount() &&
           "Assertion failed in 'SetConfigurationMatrix");
    memcpy(configuration_matrix->GetDataPtr(), A.GetDataPtr(), A.Size());
}

void Robo::UpdateTranslationMatrix()
{
    assert(configuration_matrix->GetDataPtr() != nullptr);
    assert(configuration_matrix->GetRowsCount() == number_of_elements);
    auto &A = this->configuration_matrix;

    array<Matrix, 6> T;

    for (counter i = 0; i < T.size(); i++)
    {
        T[i] = {{cos(A->elem(i, 3)), -sin(A->elem(i, 3)) * cos(A->elem(i, 1)),  sin(A->elem(i, 3)) * sin(A->elem(i, 1)), A->elem(i, 0) * cos(A->elem(i, 3))},
                {sin(A->elem(i, 3)),  cos(A->elem(i, 3)) * cos(A->elem(i, 1)), -cos(A->elem(i, 3)) * sin(A->elem(i, 1)), A->elem(i, 0) * sin(A->elem(i, 3))},
                {0,                   sin(A->elem(i, 1)),                       cos(A->elem(i, 1)),                      A->elem(i, 2)},
                {0,                   0,                                          0,                                     1}};

    }
    array<Matrix, 6> T6;
    T6[0] = T[0];
    for (size_t i = 1; i < T6.size(); i++)
    {
        T6[i] = T6[i-1] * T[i];
    }
    memcpy(translation_matrix->GetDataPtr(), T6[5].GetDataPtr(), T6[5].Size());
}

Vector3 Robo::GetEffectorPosition()
{
    return this->Effector()->GetPosition();
}

Elem *Robo::Effector() const
{
    assert(number_of_elements > 0 && robot && robot != nullptr && "Failed assertion in Robo.Effector()");
    return (robot + number_of_elements - 1);
}

void Robo::AllocateForNewRobot()
{

    configuration_matrix = new Matrix(number_of_elements, 4);
    translation_matrix = new Matrix(4, 4);
    jacobian     = new Matrix(EFFECTOR_DEGREE_OF_FREEDOM, number_of_elements - 1);
    delta_thetas = new Vector(number_of_elements - 1);
    delta_s      = new Vector(3);

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

const Vector3 Elem::GetRotationAxisLocal() const
{
    return this->rotation_axis_loc;
}

const Vector3 Elem::GetPositionLocal() const
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


