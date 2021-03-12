#include "solver.h"


Solver::Solver(Robo &manip)
{
    manip_config = new Matrix(*manip.state_matrix);
#if !WITH_BENCHMARK
    emulator = new Emulator(manip);
#endif
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
//    auto kk = delta_s.Length() > accuracy;
//    auto kk1 = delta_r.Length() > accuracy * 100;
    while ((delta_s.Length() > accuracy) || (delta_r.Length() > accuracy * 10) || !solved)
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
//        jacobian.MatrixCout("jacobian");
        jacobian_transposed.Transpose();
//        jacobian_transposed.MatrixCout("jacobian_tra");
        U = (jacobian) * jacobian_transposed;
        U.AddDiagonal(damping_constant_sq);
//        U.MatrixCout("U");
        aug_matrix.SetSubmatrix(U, 0, 0);
        aug_matrix.SetSubColumn(delta_s, aug_matrix.GetColsCount() - 1, 0);
//        aug_matrix.MatrixCout("Matrix aug");
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
        if (d_thet.q[1] > 0.5)
            d_thet.q[1] *= - 1.0;
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
            cout << "Точка " << "x(" << target.x << ")" << " y(" << target.y << ")" << " z(" << target.z << ")"
                 << "roll(" << ToDeg(target.roll) << ")" << " pitch(" << ToDeg(target.pitch) << ")" << " yaw(" << ToDeg(target.yaw) << ") " << "недостижима" << endl;
            return -1;
        }
        //не имеет решения
    }
    if (solved)
        for (int i = 0; i < JOINTS_COUNT; i++)
            res_angle.q[i] = d_thet.q[i];
    Matrix4x4 TRR[JOINTS_COUNT];
    SolveDirectKinematic(res_angle, TRR);
//    cout << "InverseKinematic result \nx = " << TRR[5](0,3) << " y = " << TRR[5](1,3) << " z = " << TRR[5](2,3)
//         << "\nroll = " << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().x)
//         << " pitch = " << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().y)
//         << " yaw = "   << ToDeg(TRR[5].ConvertToQuaternion().GetEulerAngles().z)
//         << endl;
    return 0;
}

int Solver::CalcPolynomMaxQPP(const StateVector &start_point, const StateVector &end_point, const Angles &curr_angles, Angles& res_angle, const int &t_millisec, const scalar &dt_millisec, Polynom result[])
{
//    scalar x(start_point.x), dx(0), vx(0), ax(0);
//    scalar y(start_point.y), dy(0), vy(0), ay(0);
//    scalar z(start_point.z), dz(0), vz(0), az(0);
//    scalar rol(start_point.roll),  d_rol(0);
//    scalar pit(start_point.pitch), d_pit(0);
//    scalar yaw(start_point.yaw),   d_yaw(0);
//    int dt = dt_millisec; //millisec
//    scalar dt_s = ToSec(dt_millisec);
//    Matrix4x4 buff[6];
//    Angles angles[t_millisec / dt + 1];
//    //calc max a value
//    Angles max_qpp;
//    scalar v_tcp(0.5);
//    scalar a_tcp(0.1);
//    scalar distance = Distance(end_point, start_point);
//    scalar
//            dist_x = end_point.x - start_point.x,
//            dist_y = end_point.y - start_point.y,
//            dist_z = end_point.z - start_point.z;
//    for (int i = 0; i < 6; i++)
//        max_qpp.q_pp[i] = 0.5;
//    //__________
//    SolveInverseKinematic(ACCURACY, start_point, curr_angles, angles[0]);
//    SolveDirectKinematic(angles[0], buff);
//    //change 100 to variable
//    dx = (end_point.x - start_point.x) / (t_millisec / dt);
//    dy = (end_point.y - start_point.y) / (t_millisec / dt);
//    dz = (end_point.z - start_point.z) / (t_millisec / dt);

}
int Solver::CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const Angles &curr_angles, Angles& res_angle, const int &t_millisec, const ushort &dt_millisec, Polynom result[])
{
    scalar x(start_point.x), dx(0), vx(0), ax(0);
    scalar y(start_point.y), dy(0), vy(0), ay(0);
    scalar z(start_point.z), dz(0), vz(0), az(0);
    scalar rol(start_point.roll),  d_rol(0);
    scalar pit(start_point.pitch), d_pit(0);
    scalar yaw(start_point.yaw),   d_yaw(0);
    scalar t(0);
//    int dt = dt_millisec; //millisec
    scalar dt_s = ToSec(dt_millisec);
    Matrix4x4 buff[6];
    Angles angles[500];
    Vector3 speed[500];
    Vector3 accel[500];
    //calc max a value
    Angles max_qpp;
    scalar v_tcp(0.5);
    scalar a_tcp(1);
    scalar v_curr;
    scalar a_curr;
    scalar step(0);
    scalar distance = Distance(end_point, start_point);
    scalar  dist_x_norm = (end_point.x - start_point.x) / distance,
            dist_y_norm = (end_point.y - start_point.y) / distance,
            dist_z_norm = (end_point.z - start_point.z) / distance;

    auto min_distance = distance;
    scalar t_stop;
    //__________
    SolveInverseKinematic(ACCURACY, start_point, curr_angles, angles[0]);
    SolveDirectKinematic(angles[0], buff);
    int size(0);
    bool is_last(0);
    bool breaking(false);
    bool accel_break(false);
    while (Distance(end_point, StateVector(x, y, z, rol, pit, yaw)) > 10e-5 && !is_last)
    {
        v_curr = sqrt(vx*vx + vy*vy + vz*vz);
        a_curr = sqrt(ax*ax + ay*ay + az*az);
        t_stop = v_curr / a_tcp;
        auto distance_next_step = Distance(end_point, StateVector(x + vx * dt_s + ax*dt_s*dt_s/2,
                                                                  y + vy * dt_s + ay*dt_s*dt_s/2,
                                                                  z + vz * dt_s + az*dt_s*dt_s/2,
                                                                  rol + 0, pit + 0, yaw + 0));
        auto distance = Distance(end_point, StateVector(x, y, z, rol, pit, yaw));
//        if (distance <= min_distance)
//        if (distance <= step)
//            is_last = true;
        if (!sign(vy + ay * dt_s, vy))
            is_last = true;
        step = sqrt(pow(vx * dt_s, 2) + pow(vy * dt_s, 2) + pow(vz * dt_s, 2));
        auto distance_to_stop = v_curr * t_stop - a_tcp * t_stop * t_stop * 0.5;
        //Режим торможения (если расстояние < Sтормож)
        if (/*Distance(end_point, StateVector(x, y, z, rol, pit, yaw))*/distance_next_step <= (v_curr * t_stop - a_curr * t_stop * t_stop * 0.5) && !breaking)
//        if (t > 1)
        {
            accel_break = true;
            breaking = true;
        }
        if (breaking)
        {
            //Проекции вектора ускорения на оси, считаем 1 раз. Если считать на каждом шагу, скорость будет уменьшаться не линейно
//            if (accel_break)
            {
                ax = (2 * (end_point.x - x - vx * t_stop) * (1/t_stop/t_stop));// * abs(dist_x_norm);
                ay = (2 * (end_point.y - y - vy * t_stop) * (1/t_stop/t_stop));// * abs(dist_y_norm);
                az = (2 * (end_point.z - z - vz * t_stop) * (1/t_stop/t_stop));// * abs(dist_z_norm);
                accel_break = false;
            }
//            if (Distance(StateVector(x,y,z,rol,pit,yaw), end_point) <= 2 * v_curr * dt_s/* && t_stop <= dt_s * 4*/)
//            {
//                is_last = true;
////                scalar a = Distance(StateVector(x,y,z,rol,pit,yaw), end_point) - v_curr * dt_s * 2 / dt_s / dt_s;
//                ax = 2 * ((end_point.x - x) - vx * dt_s) / dt_s / dt_s;
//                ay = 2 * ((end_point.y - y) - vy * dt_s) / dt_s / dt_s;
//                az = 2 * ((end_point.z - z) - vz * dt_s) / dt_s / dt_s;
//            }
        }
        else
        {
            ax = a_tcp * dist_x_norm;
            ay = a_tcp * dist_y_norm;
            az = a_tcp * dist_z_norm;
        }
        if (!is_last)
        {
            x += vx * dt_s + ax*dt_s*dt_s/2;
            y += vy * dt_s + ay*dt_s*dt_s/2;
            z += vz * dt_s + az*dt_s*dt_s/2;

            if (abs(vx + ax * dt_s) < abs(v_tcp * dist_x_norm)) // vx На след шаге < требуемой
                vx += ax * dt_s;
            else
                vx = v_tcp * dist_x_norm;
            if (abs(vy + ay * dt_s) < abs(v_tcp * dist_y_norm)) // vx На след шаге < требуемой
                vy += ay * dt_s;
            else
                vy = v_tcp * dist_y_norm;
            if (abs(vz + az * dt_s) < abs(v_tcp * dist_z_norm)) // vx На след шаге < требуемой
                vz += az * dt_s;
            else
                vz = v_tcp * dist_z_norm;
        }
        else
        {
            ax = 2 * ((end_point.x - x) - vx * dt_s) / dt_s / dt_s;
            ay = 2 * ((end_point.y - y) - vy * dt_s) / dt_s / dt_s;
            az = 2 * ((end_point.z - z) - vz * dt_s) / dt_s / dt_s;
            x = end_point.x;
            y = end_point.y;
            z = end_point.z;
            vx = 0;
            vy = 0;
            vz = 0;
        }


        auto p = sqrt(pow(vy * dt_s + ay*dt_s*dt_s/2, 2));
        //сделать аналогично для углов
//        rol += d_rol; pit += d_pit; yaw += d_yaw;
        SolveInverseKinematic(ACCURACY, StateVector(x, y, z, rol, pit, yaw), angles[size], angles[size+1]);

        if (size > 180)
            break;
        size++;
        t+= dt_s;
        distance = Distance(end_point, StateVector(x + vx * dt_s + ax*dt_s*dt_s/2, vy * dt_s + ay*dt_s*dt_s/2, vz * dt_s + az*dt_s*dt_s/2, rol + 0, pit + 0, yaw + 0));
        speed[size].x = vx;
        speed[size].y = vy;
        speed[size].z = vz;

        accel[size].x = vx;
        accel[size].y = vy;
        accel[size].z = vz;

    }

//    int i(0);
//    while (x != end_point.x && y != end_point.y && z != end_point.z)
//    {
//        ax = a_tcp * dist_x / distance;
//        ay = a_tcp * dist_y / distance;
//        az = a_tcp * dist_z / distance;
//        //попробовать расчет по t
//        if ((vx + ax * dt_s) <= v_tcp * dist_x)
//        {
//            vx += ax * dt_s;
//            vy += ay * dt_s;
//            vz += az * dt_s;
//    //        d_x = start_point.x + (V0x + dt * ax) + ax * dt * dt / 2;
//    //        d_y = start_point.y + (V0y + dt * ay) + ay * dt * dt / 2;
//    //        d_z = start_point.z + (V0z + dt * az) + az * dt * dt / 2;
//        }
//        else // v = vтреб
//        {
//            vx = v_tcp * (end_point.x - start_point.x) / distance;
//            vy = v_tcp * (end_point.y - start_point.y) / distance;
//            vz = v_tcp * (end_point.z - start_point.z) / distance;
//        }
//        dx = vx * dt_s;
//        dy = vy * dt_s;
//        dz = vz * dt_s;

//        d_rol = (end_point.roll  - start_point.roll)  / (t_millisec / dt);
//        d_pit = (end_point.pitch - start_point.pitch) / (t_millisec / dt);
//        d_yaw = (end_point.yaw   - start_point.yaw)   / (t_millisec / dt);
//        x += dx; y += dy; z += dz;
//        rol += d_rol; pit += d_pit; yaw += d_yaw;

//        SolveInverseKinematic(ACCURACY, StateVector(x, y, z, rol, pit, yaw), angles[i], angles[i+1]);




//        i++;
//    }

//    for (int i = 0; i < t_millisec / dt; i++)
//    {
//        x += dx; y += dy; z += dz;
//        rol += d_rol; pit += d_pit; yaw += d_yaw;
//        SolveInverseKinematic(ACCURACY, StateVector(x, y, z, rol, pit, yaw), angles[i], angles[i+1]);
//    }
    for (int i = 0; i < size/*t_millisec / dt*/; i++)
    {
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
//            angles[i].q_p[j] = (angles[i + 1].q[j] - angles[i].q[j]) / ToSec(dt);//0.5 * (ToSec(dt) * max_qpp.q_pp[j]);//(angles[i + 1].q[j] - angles[i].q[j]) / ToSec(dt);
//            angles[i].q_p[j] = angles[i + 1].q_p[j] + dt *
            auto ii = angles[i + 1].q[j] - angles[i].q[j] - angles[i].q_p[j] * dt_s;
            auto kk = dt_s * dt_s;
            angles[i].q_pp[j] =(angles[i + 1].q[j] - angles[i].q[j] - angles[i].q_p[j] * dt_s) / (dt_s * dt_s) /*0.1*/;
            //ускорение
            angles[i + 1].q_p[j] = angles[i].q_p[j] + angles[i].q_pp[j] * dt_s;
        }
    }
//    for (int i = 1; i < size/*t_millisec / dt*/; i++)
//    {
//        for (int j = 0; j < 6; j++)
//        {
//            angles[i].q_pp[j] = (angles[i + 1].q_p[j] - angles[i].q_p[j]) / ToSec(dt);//0.5 * (ToSec(dt) * max_qpp.q_pp[j]);//(angles[i + 1].q[j] - angles[i].q[j]) / ToSec(dt);
//        }
//    }
    for (int i = 0; i < size/*t_millisec / dt*/; i++)
    {

//        result[i].CalcPolynom(&angles[i], &angles[i + 1], dt);
        //_______________for polynoms________________________
//        CheckTrajectoryForLinear(result[i], start_point.GetCoord(), end_point.GetCoord(), dt, DELTA_RANGE);
        //___________________________________________________
        //_______________for angles________________________
        CheckTrajectoryForLinear(angles, start_point.GetCoord(), end_point.GetCoord(), dt_millisec, DELTA_RANGE);
        //___________________________________________________
//        SolveDirectKinematic(angles[i + 1], buff);
//        if (CheckCollisionAABB(buff))
//            CkeckCapsuleCollision();
//        if (CheckAABB(BoxAABB(0, 0, 0, 1, 1, 1), BoxAABB(-0.1, -0.1, -0.1, 0.1, 0.1, 0.1)))
//            cout << "ALAAAAAAAAAAAAARM" << endl;
    }
//    for (int i = 0; i < size/*t_millisec / dt*/; i++)
    emulator->MoveManipLinear(angles, std::size(angles), dt_millisec, accel);
    res_angle = angles[t_millisec / dt_millisec];
    return 0;
}

bool Solver::CheckTrajectoryForLinear(const Angles angles[], const Vector3 &l1, const Vector3 &l2, const ushort &dt_millisec, const scalar &max_delta)
{
    rm::Angles a;
    Matrix4x4 buff[JOINTS_COUNT];

    counter i(0);
    while (!angles[i].IsNull())
    {
//        for (int i = 0; i < JOINTS_COUNT; i++)
//        {
//            a.q[i] = angles.GetAngle(i, ToSec(cur_t));
//        }
        SolveDirectKinematic(angles[i], buff);
        auto distance = CalcDistancePointToLine(Vector3(buff[JOINTS_COUNT - 1](0,3), buff[JOINTS_COUNT - 1](1,3), buff[JOINTS_COUNT - 1](2,3)), &l1, &l2);
        if (distance > max_delta)
        {
            cout <<" ALARM BAD TRAJECTORY, DISTANCE = " << distance << endl;
            return false;
        } else
        {
//            cout << "ALL GOOOOOOOD, DISTANCE = " << distance << endl;
        }
        //            cout << "x = " << v.x << " y = " << v.y << " z = " << v.z
        //                 << "\nroll = " << ToDeg(v.roll) << " pitch = " << ToDeg(v.pitch) << " yaw " << ToDeg(v.yaw) << endl;
        i++;
    }
    return true;
}

pair<int,int> Solver::CheckCollisionAABB(const Matrix4x4 state[])
{
    //Возвращает номер двух столкнувшихс элементов (0..n)
    bool collision = false;
    int col1, col2;
    for (int i = 0; i < JOINTS_COUNT - 3; i++)
    {
        for (int j = i + 2; j < JOINTS_COUNT - 2; j++)
        {
            collision = CheckAABB(BoxAABB(state[i].at(0,3), state[i].at(1,3), state[i].at(2,3), state[i + 1].at(0,3), state[i + 1].at(1,3), state[i + 1].at(2,3)),
                                  BoxAABB(state[j].at(0,3), state[j].at(1,3), state[j].at(2,3), state[j + 1].at(0,3), state[j + 1].at(1,3), state[j + 1].at(2,3)));
            if (collision)
            {
                cout << "ALARM!!! COLLISION!!! Between " << i << " and " << j << endl;
                col1 = i;
                col2 = j;
            }
        }
    }
    return make_pair(col1, col2);
}

bool Solver::CheckAABB(const BoxAABB &box1, const BoxAABB &box2)
{
    return(box1.point_max.x > box2.point_min.x &&
           box1.point_min.x < box2.point_max.x &&
           box1.point_max.y > box2.point_min.y &&
           box1.point_min.y < box2.point_max.y &&
           box1.point_max.z > box2.point_min.z &&
           box1.point_min.z < box2.point_max.z);
}

BoxAABB::BoxAABB(const scalar &x_min, const scalar &y_min, const scalar &z_min, const scalar &x_max, const scalar &y_max, const scalar &z_max)
{
    // Плюс добавить дельту, которая будет обеспечивать ширину каждого линка
    point_max.x = x_max;
    point_max.y = y_max;
    point_max.z = z_max;
    point_min.x = x_min;
    point_min.y = y_min;
    point_min.z = z_min;
}
