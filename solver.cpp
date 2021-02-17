#include "solver.h"



Solver::Solver(Robo manip)
{
    manip_config = new Matrix(*manip.state_matrix);
    emulator = new Emulator(manip);
}

void Solver::SolveDirectKinematic(const Angles &q, Matrix4x4 TR[])
{
    const Matrix &A = *manip_config;
    T[0] = {{         cos(q.q[0]),         -sin(q.q[0]),       0,           A.at(0,0)},
            {sin(q.q[0]) * cos(0), cos(q.q[0]) * cos(0), -sin(0), -sin(0) * A.at(0,2)},
            {sin(q.q[0]) * sin(0), cos(q.q[0]) * sin(0),  cos(0),  cos(0) * A.at(0,2)},
            {                   0,                    0,       0,                   1}};
    for (counter i = 1; i < JOINTS_COUNT; i++)
    {
        T[i] = {{cos(q.q[i]),                                      -sin(q.q[i]),                 0,                   A.at(i-1,0)},
                {sin(q.q[i]) * cos(A.at(i-1,1)), cos(q.q[i]) * cos(A.at(i-1,1)), -sin(A.at(i-1,1)), -sin(A.at(i-1,1)) * A.at(i,2)},
                {sin(q.q[i]) * sin(A.at(i-1,1)), cos(q.q[i]) * sin(A.at(i-1,1)),  cos(A.at(i-1,1)),  cos(A.at(i-1,1)) * A.at(i,2)},
                {                                0,                                 0,                 0,                             1}};
    }
    TR[0] = T[0];
    TR[1] = T[0] * T[1];
    TR[2] = T[0] * T[1] * T[2];
    TR[3] = T[0] * T[1] * T[2] * T[3];
    TR[4] = T[0] * T[1] * T[2] * T[3] * T[4];
    TR[5] = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
}

int Solver::SolveInverseKinematic(const scalar &accuracy, const StateVector &target, const Angles &curr_angle, Angles &res_angle)
{
    Quaternion end_eff_target_ori;
    Quaternion end_eff_world_ori;
    Quaternion deltaQ;
    Matrix4x4 TR[JOINTS_COUNT];
    Vector3 ang_vel;
    Vector3 delta_r;
    Vector3 delta_s;
    Vector3 axis;
    Matrix jacobian(EFFECTOR_DEGREE_OF_FREEDOM,6);
    Matrix jacobian_transposed(EFFECTOR_DEGREE_OF_FREEDOM, 6);
    Matrix U(EFFECTOR_DEGREE_OF_FREEDOM, EFFECTOR_DEGREE_OF_FREEDOM);
    Matrix aug_matrix(U.GetRowsCount(), U.GetColsCount() + 1);
    Angles d_thet = curr_angle;
    Vector delta_t(EFFECTOR_DEGREE_OF_FREEDOM);
    Vector delta_thetas(JOINTS_COUNT);
    counter ctr(0);
    scalar angle(0);

    bool solved = false;
    SolveDirectKinematic(curr_angle, TR);

    end_eff_world_ori = TR[JOINTS_COUNT - 1].ConvertToQuaternion();
    end_eff_target_ori.EulerToQuat(target.GetAngles());
    end_eff_world_ori.Invert();
    deltaQ = end_eff_target_ori * end_eff_world_ori;
    angle = deltaQ.GetAngle();
    axis = deltaQ.GetAxis();
    if (angle > M_PI)
        angle -= 2.0 * M_PI;
    else if (angle < - M_PI)
        angle += 2.0 * M_PI;
    ang_vel = axis.Normalize() * angle;
    delta_r = ang_vel * 1;
    delta_s.x = target.x - TR[5](0,3);
    delta_s.y = target.y - TR[5](1,3);
    delta_s.z = target.z - TR[5](2,3);
    auto kk = delta_s.Length() > accuracy;
    auto kk1 = delta_r.Length() > accuracy * 100;
    while ((delta_s.Length() > accuracy) /*|| (delta_r.Length() > 0.1)*/)
    {
//        cout << "delta_r " << delta_r.Length() << " delta_s " << delta_s.Length() << endl;
        solved = true;
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            jacobian.SetSubColumn(Vector3(TR[i](0,2), TR[i](1,2), TR[i](2,2)) * (Vector3({TR[5](0,3), TR[5](1,3), TR[5](2,3)}) - Vector3({TR[i](0,3), TR[i](1,3), TR[i](2,3)})), i, 0);
#ifdef POSITION_AND_ROTATION
            jacobian.SetSubColumn(Vector3(TR[i](0,2), TR[i](1,2), TR[i](2,2)), i, 3);
#endif
        }
        //calc Jacobian for current configuration
        jacobian_transposed = jacobian;
        jacobian.MatrixCout("jacobian");
        jacobian_transposed.Transpose();
        jacobian_transposed.MatrixCout("jacobian_tra");
        U = (jacobian) * jacobian_transposed;
        U.AddDiagonal(damping_constant_sq);
        U.MatrixCout("U");
        aug_matrix.SetSubmatrix(U, 0, 0);
        aug_matrix.SetSubColumn(delta_s, aug_matrix.GetColsCount() - 1, 0);
        aug_matrix.MatrixCout("Matrix aug");
#ifdef POSITION_AND_ROTATION
        aug_matrix.SetSubColumn(delta_r, aug_matrix.GetColsCount() - 1, 3);
#endif
        aug_matrix.SolveGauss();
        aug_matrix.GetTheRoots(delta_t);
        delta_thetas = jacobian_transposed * delta_t;

        scalar max = delta_thetas.MaxAbs();
        if (max > MaxAngleDls)
        {
            delta_thetas *= MaxAngleDls/max;
        }
        for (int i = 0; i < JOINTS_COUNT; i++)
            d_thet.q[i] += delta_thetas(i);

        SolveDirectKinematic(d_thet, TR);

        end_eff_world_ori = TR[5].ConvertToQuaternion();
        end_eff_target_ori.EulerToQuat(target.GetAngles());
        end_eff_world_ori.Invert();
        deltaQ = end_eff_target_ori * end_eff_world_ori;
        angle = deltaQ.GetAngle();
        axis = deltaQ.GetAxis();
        if (angle > M_PI)
            angle -= 2.0 * M_PI;
        else if (angle < - M_PI)
            angle += 2.0 * M_PI;
        ang_vel = axis.Normalize() * angle;
        delta_r = ang_vel * 1;
        delta_s.x = target.x - TR[5](0,3);
        delta_s.y = target.y - TR[5](1,3);
        delta_s.z = target.z - TR[5](2,3);
        ctr++;
        if (ctr > MAX_KINEMATIC_ITERS)
        {
            solved = false;
            cout << "Точка " << "x(" << target.x << ")" << " y(" << target.y << ")" << " z(" << target.z << ") недостижима" << endl;
            return -1;
        }
        //не имеет решения
    }
    if (solved)
        for (int i = 0; i < JOINTS_COUNT; i++)
            res_angle.q[i] = d_thet.q[i];
    Matrix4x4 TRR[JOINTS_COUNT];
    SolveDirectKinematic(res_angle, TRR);
    cout << "InverseKinematic result \nx = " << TRR[5](0,3) << " y = " << TRR[5](1,3) << " z = " << TRR[5](2,3)
         << "\nroll = " << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().x)
         << " pitch = " << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().y)
         << " yaw = "   << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().z)
         << endl;
    return 0;
}

int Solver::CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const Angles &curr_angles, const int &t_millisec, const scalar &max_range, Polynom result[])
{
    scalar x(start_point.x), d_x(0);
    scalar y(start_point.y), d_y(0);
    scalar z(start_point.z), d_z(0);
    scalar rol(start_point.roll),  d_rol(0);
    scalar pit(start_point.pitch), d_pit(0);
    scalar yaw(start_point.yaw),   d_yaw(0);
    Angles start_angles;
    Angles new_angle;
    int dt = 50; //millisec
    Matrix4x4 buff[6];

    SolveInverseKinematic(ACCURACY, start_point, curr_angles, start_angles);
    new_angle = start_angles;
    SolveDirectKinematic(start_angles, buff);
    //change 100 to variable
    d_x = (end_point.x - start_point.x) / (t_millisec / dt);
    d_y = (end_point.y - start_point.y) / (t_millisec / dt);
    d_z = (end_point.z - start_point.z) / (t_millisec / dt);
    d_rol = (end_point.roll  - start_point.roll)  / (t_millisec / dt);
    d_pit = (end_point.pitch - start_point.pitch) / (t_millisec / dt);
    d_yaw = (end_point.yaw   - start_point.yaw)   / (t_millisec / dt);
    for (int i = 0; i < t_millisec / dt; i++)
    {
        x += d_x; y += d_y; z += d_z;
        rol += d_rol; pit += d_pit; yaw += d_yaw;
        SolveInverseKinematic(ACCURACY, StateVector(x, y, z, rol, pit, yaw), start_angles, new_angle);
//        SolveDirectKinematic(new_angle, buff);
//        cout << "Вышли в эту точку \nx = " << buff[5](0,3) << " y = " << buff[5](1,3) << " z = " << buff[5](2,3)
//             << "\nroll = " << ToDeg(buff[5].ConvertToQuaternion().GetEulerAngles().x)
//             << " pitch = " << ToDeg(buff[5].ConvertToQuaternion().GetEulerAngles().y)
//             << " yaw = "   << ToDeg(buff[5].ConvertToQuaternion().GetEulerAngles().z)
//             << endl;
        result[i].CalcPolynomLine(&start_angles, &new_angle, dt);
        start_angles = new_angle;
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
//            start_angles.q_p[j] = result[i].GetSpeed(j, i * dt + dt);
//            start_angles.q_pp[j] = result[i].GetAcceleration(j, i * dt + dt);
        }
        emulator->MoveManip(result[i], dt);
    }
}
