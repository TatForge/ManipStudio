#include "solver.h"


Solver::Solver(Robo &robot)
{
    manip_config = new Matrix(*robot.state_matrix);
    max_joint_moments = robot.GetMaxJointMoments();
    m_meshes.resize(9);
    m_mass.resize(6);
    m_main_IKData = new IKData;
    m_meshes[0].ReadStl("STLmodels/base.stl");
    m_meshes[1].ReadStl("STLmodels/joint2.stl");
    m_meshes[2].ReadStl("STLmodels/link3.stl");
    m_meshes[3].ReadStl("STLmodels/joint3.stl");
    m_meshes[4].ReadStl("STLmodels/link3.stl");
    m_meshes[5].ReadStl("STLmodels/joint4.stl");
    m_meshes[6].ReadStl("STLmodels/joint5.stl");
    m_meshes[7].ReadStl("STLmodels/joint6.stl");
    m_meshes[8].ReadStl("STLmodels/cube0.5m.stl");
    m_meshes[8].MoveMesh(Quaternion(0, 0, 0, 1), Vector3(SMESH + 0.05, SMESH - 0.05, 0));
    m_meshes_buff = m_meshes;
//    m_inertion[0] = {0.000588, 0.000587, 0.000929, -1.299e-8, 2.44e-8, 1.79e-6};
//    m_inertion[1] = {0.000928, 0.001512, 0.000937, 1.86e-5,  1.55e-10, 5.34e-12};
//    m_inertion[2] = {0.001759, 0.000747, 0.001913, 2.32e-8,  6.37e-9,  1.68e-6};
//    m_inertion[3] = {0.000198, 0.000479, 0.002059, 3.16e-11, 5.59e-10, 1.59e-7};ы
//    m_inertion[4] = {9.826e-5, 0.000128, 0.000114, -2.74e-9, 4.41e-9,  5.82e-7};
//    m_inertion[5] = {5.529e-5, 5.359e-5, 8.415e-5, 1.99e-10, 2.53e-9,  1.85e-7};

    m_inertion[0] = {0.0120, 0.0117, 0.0032, 0, 0, 0};
    m_inertion[1] = {0.0055, 0.4014, 0.3983, 0, 0, -0.0276};
    m_inertion[2] = {0.0050, 0.2629, 0.2596, 0, 0, 0.0228};
    m_inertion[3] = {0.0007, 0.0127, 0.0126, 0, 0, 0};
    m_inertion[4] = {0.0033, 0.0033, 0.006, 0, 0, 0};
    m_inertion[5] = {0.0002, 0.0002, 0.0001, 0, 0, 0};
    scalar m_gruz(5);
    Vector3 l_gruz(0.0, 0.0, 0.0);
    m_mass[0] = 2.7600;
    m_mass[1] = 2.9325;
    m_mass[2] = 2.5773;
    m_mass[3] = 1.6183;
    m_mass[4] = 1.3935;
    m_mass[5] = 0.2498 + m_gruz;
    m_cm[0] = Vector({0,      -0.0046, -0.0467, 1});
    m_cm[1] = Vector({0.3251,  0,      -0.0204, 1});
    m_cm[2] = Vector({0.2869,  0,       0.0235, 1});
    m_cm[3] = Vector({0.0757,  0,       0.0011, 1});
    m_cm[4] = Vector({-0.0023, 0,       0.0364, 1});
    m_cm[5] = Vector({ 0.0001 + l_gruz.x, 0.0 + l_gruz.y, 0.0193 + l_gruz.z, 1});

    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        m_ji[i] = {{(-m_inertion[i].xx() + m_inertion[i].yy() + m_inertion[i].zz())/2, m_inertion[i].xy(), m_inertion[i].xz(), m_mass[i] * m_cm[i](0)},
                   {m_inertion[i].xy(), (m_inertion[i].xx() - m_inertion[i].yy() + m_inertion[i].zz())/2,  m_inertion[i].yz(), m_mass[i] * m_cm[i](1)},
                   {m_inertion[i].xz(), m_inertion[i].yz(), (m_inertion[i].xx() + m_inertion[i].yy() - m_inertion[i].zz())/2,  m_mass[i] * m_cm[i](2)},
                   {m_mass[i] * m_cm[i](0),  m_mass[i] * m_cm[i](1), m_mass[i] * m_cm[i](2), m_mass[i]}};
    }
    m_moments_counter = 0;
    m_collisions_counter = 0;

#if !WITH_BENCHMARK
    m_emulator = new Emulator(robot);
#endif
    uint vector_size = 500;
    m_moments_v.resize(vector_size);
    m_angles_v.resize(vector_size);
    m_speed_v.resize(vector_size);
    m_accel_v.resize(vector_size);
#if MULTITHREAD

    m_closing = 0;
    m_dynamic_IKData = new IKData;
    m_dynamic_thread = new thread(&Solver::SolveForwardDynamicMultithread, this);
    m_collision_thread = new thread(&Solver::CheckCollisionMultithread, this);
    m_dynamic_thread->detach();
    m_collision_thread->detach();
#endif
}

Solver::~Solver()
{
    delete manip_config;
    delete m_main_IKData;
    delete m_emulator;

#if MULTITHREAD
    delete m_dynamic_IKData;
    m_closing = 1;
    m_collision_cv.notify_all();
    m_dynamic_cv.notify_all();
#endif
}
void Solver::SolveDirectKinematic(const Angles &q, Matrix4x4 TR[], Matrix4x4 *T) noexcept
{
    const Matrix &A = *manip_config;
    if (T == nullptr)
        T = m_T;

#if MULTITHREAD
    //    Matrix4x4 T[6];
#endif
    T[0] = {{         cos(q.q[0]),         -sin(q.q[0]),       0,           A.at(0,0)},
            {sin(q.q[0]) * cos(0), cos(q.q[0]) * cos(0), -sin(0), -sin(0) * A.at(0,2)},
            {sin(q.q[0]) * sin(0), cos(q.q[0]) * sin(0),  cos(0),  cos(0) * A.at(0,2)},
            {                   0,                    0,       0,                   1}};
    for (counter i = 1; i < JOINTS_COUNT; i++)
    {
        T[i] = {{cos(q.q[i]),                                      -sin(q.q[i]),                 0,                   A.at(i-1,0)},
                {sin(q.q[i]) * cos(A.at(i-1,1)), cos(q.q[i]) * cos(A.at(i-1,1)), -sin(A.at(i-1,1)), -sin(A.at(i-1,1)) * A.at(i,2)},
                {sin(q.q[i]) * sin(A.at(i-1,1)), cos(q.q[i]) * sin(A.at(i-1,1)),  cos(A.at(i-1,1)),  cos(A.at(i-1,1)) * A.at(i,2)},
                {                             0,                              0,                 0,                             1}};
    }

    TR[0] = T[0];
    TR[1] = T[0] * T[1];
    TR[2] = T[0] * T[1] * T[2];
    TR[3] = T[0] * T[1] * T[2] * T[3];
    TR[4] = T[0] * T[1] * T[2] * T[3] * T[4];
    TR[5] = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
}

void Solver::SolveInverseKinematic(const StateVector &target, const Angles &curr_angle, Angles &res_angle, IKData *data)
{
    if (data == nullptr)
        data = m_main_IKData;

    data->d_thet = curr_angle;
    data->ctr = 0;
    data->angle = 0;
    bool solved = false;

    SolveDirectKinematic(curr_angle, data->TR);

    data->end_eff_world_ori = data->TR[JOINTS_COUNT - 1].ConvertToQuaternion();
    data->end_eff_target_ori.EulerToQuat(target.GetAngles());
    data->end_eff_world_ori.Invert();
    data->deltaQ = data->end_eff_target_ori * data->end_eff_world_ori;
    data->angle = data->deltaQ.GetAngle();
    data->axis  = data->deltaQ.GetAxis();
    if (data->angle > M_PI)
        data->angle -= 2.0 * M_PI;
    else if (data->angle < - M_PI)
        data->angle += 2.0 * M_PI;
    data->ang_vel = data->axis.Normalize() * data->angle;
    data->delta_r = data->ang_vel * 1;
    data->delta_s.x = target.x - data->TR[5](0,3);
    data->delta_s.y = target.y - data->TR[5](1,3);
    data->delta_s.z = target.z - data->TR[5](2,3);
    while ((data->delta_s.Length() > ACCURACY) || (data->delta_r.Length() > ACCURACY * 10) || !solved)
    {
        solved = true;
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            data->jacobian.SetSubColumn(Vector3(data->TR[i](0,2), data->TR[i](1,2), data->TR[i](2,2)) * (Vector3({data->TR[5](0,3), data->TR[5](1,3), data->TR[5](2,3)}) - Vector3({data->TR[i](0,3), data->TR[i](1,3), data->TR[i](2,3)})), i, 0);
            data->jacobian.SetSubColumn(Vector3(data->TR[i](0,2), data->TR[i](1,2), data->TR[i](2,2)), i, 3);
        }
        //calc Jacobian for current configuration
        data->jacobian_transposed = data->jacobian;
        data->jacobian_transposed.Transpose();
        data->U = (data->jacobian) * data->jacobian_transposed;
        data->U.AddDiagonal(damping_constant_sq);
        data->aug_matrix.SetSubmatrix(data->U, 0, 0);
        data->aug_matrix.SetSubColumn(data->delta_s, data->aug_matrix.GetColsCount() - 1, 0);
        data->aug_matrix.SetSubColumn(data->delta_r, data->aug_matrix.GetColsCount() - 1, 3);
        data->aug_matrix.SolveGauss();
        data->aug_matrix.GetTheRoots(data->delta_t);
        data->delta_thetas = data->jacobian_transposed * data->delta_t;

        scalar max = data->delta_thetas.MaxAbs();
        if (max > MaxAngleDls)
        {
            data->delta_thetas *= MaxAngleDls/max;
        }
        for (int i = 0; i < JOINTS_COUNT; i++)
            data->d_thet.q[i] += data->delta_thetas(i);
        if (data->d_thet.q[1] > 0.5)
            data->d_thet.q[1] *= - 1.0;
        SolveDirectKinematic(data->d_thet, data->TR);

        data->end_eff_world_ori = data->TR[5].ConvertToQuaternion();
        data->end_eff_target_ori.EulerToQuat(target.GetAngles());
        data->end_eff_world_ori.Invert();
        data->deltaQ = data->end_eff_target_ori * data->end_eff_world_ori;
        data->angle = data->deltaQ.GetAngle();
        data->axis = data->deltaQ.GetAxis();
        if (data->angle > M_PI)
            data->angle -= 2.0 * M_PI;
        else if (data->angle < - M_PI)
            data->angle += 2.0 * M_PI;
        data->ang_vel = data->axis.Normalize() * data->angle;
        data->delta_r = data->ang_vel * 1;
        data->delta_s.x = target.x - data->TR[5](0,3);
        data->delta_s.y = target.y - data->TR[5](1,3);
        data->delta_s.z = target.z - data->TR[5](2,3);
        data->ctr++;
        if (data->ctr > MAX_KINEMATIC_ITERS)
        {
            solved = false;
#if WITH_BENCHMARK
            cout << " cant move to this point" << endl;
#else
            throw std::runtime_error("<<InverseKinematic>> Точка недостижима: x(" + to_string(target.x) + ") y(" + to_string(target.y) +
                                     ") z(" + to_string(target.z) + ") roll(" + to_string(ToDeg(target.roll)) + ") pitch("
                                     + to_string(ToDeg(target.pitch)) + ") yaw(" + to_string(ToDeg(target.yaw)) + ")");
#endif
        }
    }
    if (solved)
        for (int i = 0; i < JOINTS_COUNT; i++)
            res_angle.q[i] = data->d_thet.q[i];
    Matrix4x4 TRR[JOINTS_COUNT];
    SolveDirectKinematic(res_angle, TRR);
}

void Solver::MoveL(const StateVector &start_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const Angles &curr_angles,
                   /*Angles& res_angle,*/ const ushort &dt_millisec, vector<float> &results)
{
    m_moments_v.resize(vector_size);
    m_angles_v.resize(vector_size);
    m_speed_v.resize(vector_size);
    m_accel_v.resize(vector_size);
    scalar x(start_point.x), vx(0), ax(0);
    scalar y(start_point.y), vy(0), ay(0);
    scalar z(start_point.z), vz(0), az(0);
    scalar rol(start_point.roll);
    scalar pit(start_point.pitch);
    scalar yaw(start_point.yaw);
    scalar t(0);
    scalar dt_s = ToSec(dt_millisec);
    Matrix4x4 buff[6];
    m_moments_counter = 0;
    m_collisions_counter = 0;
    Angles max_qpp;
    scalar v_curr;
    scalar a_curr;
    scalar distance = Distance(end_point, start_point);
    scalar  dist_x_norm = (end_point.x - start_point.x) / distance,
            dist_y_norm = (end_point.y - start_point.y) / distance,
            dist_z_norm = (end_point.z - start_point.z) / distance,
            dist_rol = (end_point.roll - start_point.roll),
            dist_pit = (end_point.pitch - start_point.pitch),
            dist_yaw = (end_point.yaw  - start_point.yaw);
    scalar dist_curr;
    scalar t_stop;
    //__________

    SolveInverseKinematic(start_point, curr_angles, m_angles_v[0]);
    SolveDirectKinematic(m_angles_v[0], buff);

    uint size(0);
    bool is_last(0);
    bool breaking(false);
    scalar distance_next_step;
    while (Distance(end_point, StateVector(x, y, z, rol, pit, yaw)) > 10e-6 && !is_last)
    {
        if (size > 0)
        {
#if MULTITHREAD
//            //Расчет динамики и коллизий на предыдущем шаге
            m_dynamic_notified = true;
            m_dynamic_cv.notify_one();
            m_collision_notified = true;
            m_collision_cv.notify_one();
#endif
        }
        v_curr = sqrt(vx*vx + vy*vy + vz*vz);
        a_curr = sqrt(ax*ax + ay*ay + az*az);
        t_stop = v_curr / a_tcp;
        distance_next_step = Distance(end_point, StateVector(x + vx * dt_s + ax*dt_s*dt_s/2,
                                                                  y + vy * dt_s + ay*dt_s*dt_s/2,
                                                                  z + vz * dt_s + az*dt_s*dt_s/2,
                                                                  rol + 0, pit + 0, yaw + 0));
        if (!sign(vy + ay * dt_s, vy))
            is_last = true;
        //Режим торможения (если расстояние < Sтормож)
        if (distance_next_step <= (v_curr * t_stop - a_curr * t_stop * t_stop * 0.5) && !breaking)
        {
            breaking = true;
        }
        if (breaking)
        {
            ax = (2 * (end_point.x - x - vx * t_stop) / t_stop/t_stop);// * abs(dist_x_norm);
            ay = (2 * (end_point.y - y - vy * t_stop) / t_stop/t_stop);// * abs(dist_y_norm);
            az = (2 * (end_point.z - z - vz * t_stop) / t_stop/t_stop);// * abs(dist_z_norm);
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
            dist_curr = Distance(start_point, StateVector(x,y,z,rol,pit,yaw));
            rol = start_point.roll  + dist_rol * (dist_curr/distance);
            pit = start_point.pitch + dist_pit * (dist_curr/distance);
            yaw = start_point.yaw   + dist_yaw * (dist_curr/distance);

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
            rol = end_point.roll;
            pit = end_point.pitch;
            yaw = end_point.yaw;
        }
        if (size + 1 >= m_angles_v.size())
            VectorsResize(size);

        SolveInverseKinematic(StateVector(x, y, z, rol, pit, yaw), m_angles_v[size], m_angles_v[size+1]);
        m_speed_v[size + 1].x = vx;
        m_speed_v[size + 1].y = vy;
        m_speed_v[size + 1].z = vz;
        m_accel_v[size].x = ax;
        m_accel_v[size].y = ay;
        m_accel_v[size].z = az;
        size++;
        t+= dt_s;
    }
    for (counter i = 0; i < size; i++)
    {
        for (uint j = 0; j < JOINTS_COUNT; j++)
        {
            m_angles_v[i].q_pp[j] = (m_angles_v[i + 1].q[j] - m_angles_v[i].q[j] - m_angles_v[i].q_p[j] * dt_s) / (dt_s * dt_s);
            m_angles_v[i + 1].q_p[j] = m_angles_v[i].q_p[j] + m_angles_v[i].q_pp[j] * dt_s;
        }
    }
    CalcCollisionsDynamics(size, dt_millisec);
    m_angles_v.resize(size + 1);
    m_speed_v.resize(size + 1);
    m_accel_v.resize(size + 1);
#if WITH_WINDOW
    for (counter i = 0; i < size; i++)
    {
        m_emulator->moments_window.PlotCoordinates(m_moments_v[i].tao[0], m_moments_v[i].tao[1], m_moments_v[i].tao[2],
                                                   m_moments_v[i].tao[3], m_moments_v[i].tao[4], m_moments_v[i].tao[5], dt_millisec);
    }
#endif
    for (counter j = 1; j < JOINTS_COUNT; j++)
    {
        m_angles_v[size].q_pp[j] = (m_angles_v[size].q[j] - m_angles_v[size - 1].q[j] - m_angles_v[size - 1].q_p[j] * dt_s) / (dt_s * dt_s);
        m_angles_v[size].q_p[j] = 0;
    }
    for (counter i = 0; i < size; i++)
    {
        CheckTrajectoryForLinear(m_angles_v[i], start_point.GetCoord(), end_point.GetCoord(), DELTA_RANGE);
    }
#if !WITH_BENCHMARK
#if WITH_WINDOW
    m_emulator->moments_window.PlotCoordinates(m_moments.tao[0], m_moments.tao[1],m_moments.tao[2],m_moments.tao[3],m_moments.tao[4],m_moments.tao[5], dt_millisec, true);
    m_emulator->MoveManipLinear(m_angles_v, m_speed_v, dt_millisec);
//    res_angle = m_angles_v.back();
#endif
#endif
    AnglesVectorToVector(size, results);
}

bool Solver::CheckTrajectoryForLinear(const Angles angles, const Vector3 &l1, const Vector3 &l2, const scalar &max_delta)
{
    rm::Angles a;
    Matrix4x4 buff[JOINTS_COUNT];
    SolveDirectKinematic(angles, buff);
    auto distance = CalcDistancePointToLine(Vector3(buff[JOINTS_COUNT - 1](0,3), buff[JOINTS_COUNT - 1](1,3), buff[JOINTS_COUNT - 1](2,3)), &l1, &l2);
    if (distance > max_delta)
    {
        cout <<" ALARM BAD TRAJECTORY, DISTANCE = " << distance << endl;
        throw QString("cant reach point") + QString::number(distance);
        return false;
    }
    return true;
}

bool Solver::MoveMeshes(const Matrix4x4 TR[]) noexcept
{
    std::vector<Vector3>v_move;
    std::vector<Quaternion> quat_move;
    v_move.resize(7);
    quat_move.resize(6);
    v_move[0] = Vector3(0, 0, 0);
    v_move[1] = TR[0].ConvertToVector3();
    v_move[2] = TR[1].ConvertToVector3();
    v_move[3] = TR[2].ConvertToVector3();
    v_move[4] = TR[3].ConvertToVector3();
    v_move[5] = TR[4].ConvertToVector3();
    v_move[6] = TR[5].ConvertToVector3();

    quat_move[0] = TR[0].ConvertToQuaternion();
    quat_move[1] = TR[1].ConvertToQuaternion();
    quat_move[2] = TR[2].ConvertToQuaternion();
    quat_move[3] = TR[3].ConvertToQuaternion();
    quat_move[4] = TR[4].ConvertToQuaternion();
    quat_move[5] = TR[5].ConvertToQuaternion();

    //пересчет точек тут же
    m_meshes_buff = m_meshes;
    m_meshes_buff[0].MoveMesh(quat_move[0], Vector3(0, 0, 0));
    m_meshes_buff[1].MoveMesh(quat_move[0], v_move[1]);
    m_meshes_buff[2].MoveMesh(quat_move[1], v_move[2]);
    m_meshes_buff[3].MoveMesh(quat_move[2], v_move[3]);
    m_meshes_buff[4].MoveMesh(quat_move[2], v_move[3]);
    m_meshes_buff[5].MoveMesh(quat_move[3], v_move[4]);
    m_meshes_buff[6].MoveMesh(quat_move[4], v_move[4]);
    m_meshes_buff[7].MoveMesh(quat_move[5], v_move[5]);
    return true;
}

void Solver::CalcUMatrix() noexcept
{
    for (counter k = 0; k < 6; k++)
    {
        for (counter j = k; j < 6; j++)
        {
            U[k*6 + j].MakeIdentity();
            //Считается Ujk для конкретного i,j,k
            for (int ctr = 0; ctr <= j; ctr++)
            {
                if (ctr == k)
                    U[k*6 + j] = U[k*6 + j] * m_T[ctr] * cq;
                else
                {
                    U[k*6 + j] = U[k*6 + j] * m_T[ctr];
                }
            }
        }
    }
}

void Solver::CalcUMatrix(const Angles &a)
{
    const Matrix &A = *manip_config;
    m_T[0] = {{       cos(a.q[0]),         -sin(a.q[0]),       0,           A.at(0,0)},
              {sin(a.q[0]) * cos(0), cos(a.q[0]) * cos(0), -sin(0), -sin(0) * A.at(0,2)},
              {sin(a.q[0]) * sin(0), cos(a.q[0]) * sin(0),  cos(0),  cos(0) * A.at(0,2)},
              {                   0,                    0,       0,                   1}};
    for (counter i = 1; i < JOINTS_COUNT; i++)
    {
        m_T[i] = {{cos(a.q[i]),                                      -sin(a.q[i]),                 0,                     A.at(i-1,0)},
                  {sin(a.q[i]) * cos(A.at(i-1,1)), cos(a.q[i]) * cos(A.at(i-1,1)), -sin(A.at(i-1,1)), -sin(A.at(i-1,1)) * A.at(i,2)},
                  {sin(a.q[i]) * sin(A.at(i-1,1)), cos(a.q[i]) * sin(A.at(i-1,1)),  cos(A.at(i-1,1)),  cos(A.at(i-1,1)) * A.at(i,2)},
                  {                              0,                               0,                 0,                            1}};
    }
    for (counter k = 0; k < 6; k++)
    {
        for (counter j = k; j < 6; j++)
        {
            U[k*6 + j].MakeIdentity();
            //Считается Ujk для конкретного i,j,k
            for (int ctr = 0; ctr <= j; ctr++)
            {
                if (ctr == k)
                    U[k*6 + j] = U[k*6 + j] * m_T[ctr] * cq;
                else
                {
                    U[k*6 + j] = U[k*6 + j] * m_T[ctr];
                }
            }
        }
    }
}

bool Solver::CheckMaxMoments()
{
    for (int joint_number = 0; joint_number < 6; joint_number++)
    {
        if (abs(m_moments.tao[joint_number]) > max_joint_moments[joint_number])
        {
#if WITH_WINDOW
            m_emulator->OneIterMoveLinear(m_angles_v[m_moments_counter], m_angles_v[m_moments_counter],  DT_MILLISEC, true, m_accel_v[m_collisions_counter]);
#endif
            throw std::runtime_error("Calculated moment of the " + to_string(joint_number) + "'s engine is greated than the maximum possible[" + to_string(max_joint_moments[joint_number]) + "]");
            return true;
        }
    }
    return false;
}

void Solver::SolveForwardDynamicMultithread()
{
#if MULTITHREAD
    //    cout << "Solve Forward Dynamic started" << endl;
    unique_lock<mutex> lock(m_dynamic_mtx);
    while(!m_closing)
    {
        while(!m_dynamic_notified)
        {
            m_dynamic_cv.wait(lock);
        }
        CalcUMatrix(m_angles_v[m_moments_counter]);
        SolveForwardDynamic(m_angles_v[m_moments_counter]);
        m_moments_v[m_moments_counter] = m_moments;
#if WITH_WINDOW
        //        m_emulator->moments_window.PlotCoordinates(m_moments_vector[m_moments_counter].tao[0], m_moments_vector[m_moments_counter].tao[1],
        //                                                   m_moments_vector[m_moments_counter].tao[2], m_moments_vector[m_moments_counter].tao[3],
        //                                                   m_moments_vector[m_moments_counter].tao[4], m_moments_vector[m_moments_counter].tao[5], dt_millisec);
#endif
        m_dynamic_notified = false;
        m_dynamic_cv.wait(lock);
    }
#endif
}

void Solver::CheckCollisionMultithread()
{
#if MULTITHREAD && WITH_COLLISIONS
    Matrix4x4 buff1[6];
    unique_lock<mutex> lock(m_collision_mtx);
    Matrix4x4 T[6];
    while(!m_closing)
    {
        while(!m_collision_notified)
        {
            m_collision_cv.wait(lock);
        }
        SolveDirectKinematic(m_angles_v[m_collisions_counter], buff1, T);
        if (MoveMeshes(buff1))
        {
            CheckCollision();
        }
        m_collisions_counter++;
        m_collision_notified = false;
        m_collision_cv.wait(lock);
    }
#endif
}

bool Solver::SolveForwardDynamic(const Angles &angles)
{
#if MULTITHREAD
//    cout << "Solve Forward Dynamic started" << endl;
#endif
    CalcUMatrix();
    m_moments.MakeNull();
    scalar buff(0);
    int ctr(0);
    for (counter i = 0; i < 6; i++)
    {
        for (counter j = i; j < 6; j++)
        {
            m_moments.p[i] += - m_mass[j] * m_gv.MultiplyTranspose(U[i*6 + j].MultiplyTranspose(m_cm[j]));
        }
        ctr++;
    }
    //Расчет D для i-го элемента манипулятора
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = 0; k < 6; k++)
        {
            buff = 0;
            for (counter j = RoboMath::max(i, k); j < 6; j++)
            {
                buff += Trace(U[k*6 + j] * m_ji[k].MultiplyTranspose(U[i*6 + j]));
            }
            buff *= angles.q_pp[k];
            m_moments.dv[i] += buff;
        }
    }
    Matrix4x4 u_buff;
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = 0; k < 6; k++)
        {
            for (counter m = 0; m < 6; m++)
            {
                buff = 0;
                for (counter j = i; j < 6; j++)
                {
                    u_buff.MakeIdentity();
                    for (counter ctr = 0; ctr < j; ctr++)
                    {
                        if (ctr == k && ctr == m)
                            u_buff = u_buff * cq_sq * m_T[ctr];
                        else if (ctr == k || ctr == m)
                            u_buff = u_buff * cq * m_T[ctr];
                        else
                            u_buff = u_buff * m_T[ctr];
                    }
                    buff += Trace(u_buff * m_ji[i].MultiplyTranspose(U[i*6 + j])) * angles.q_p[k] * angles.q_p[m];
                }
                m_moments.hikm[i] += buff;
            }
        }
    }
    for ( int i = 0; i < 6; i++)
    {
        m_moments.tao[i] = m_moments.p[i] + m_moments.dv[i] + m_moments.hikm[i];
    }
    m_moments_counter++;
    CheckMaxMoments();
    return true;
}

void Solver::VisualizeMoments(const scalar &dt_millisec)
{
#if WITH_WINDOW
    for (counter i = 0; i < m_moments_counter; i++)
    {
        if (i == 0)
            m_emulator->moments_window.PrintResult(m_moments_v[i].tao[0], m_moments_v[i].tao[1],
                    m_moments_v[i].tao[2], ToRad(m_moments_v[i].tao[3]),
                    ToRad(m_moments_v[i].tao[4]), ToRad(m_moments_v[i].tao[5]));
        m_emulator->moments_window.PlotCoordinates(m_moments_v[i].tao[0], m_moments_v[i].tao[1],
                m_moments_v[i].tao[2], m_moments_v[i].tao[3],
                m_moments_v[i].tao[4], m_moments_v[i].tao[5], dt_millisec);
        if (i == m_moments_counter - 1)
            m_emulator->moments_window.PlotCoordinates(m_moments_v[i].tao[0], m_moments_v[i].tao[1],
                    m_moments_v[i].tao[2], m_moments_v[i].tao[3],
                    m_moments_v[i].tao[4], m_moments_v[i].tao[5], dt_millisec, true);
    }
#endif
}

void Solver::AnglesVectorToVector(const int size, vector<float> &results)
{
    results.clear();
    results.resize(size * sizeof(Angles) / sizeof(scalar));
    int s = sizeof(Angles) / sizeof(scalar);
    for (counter i = 0; i < size; i++)
    {
        for (counter j = 0; j < 6; j++)
        {
            results[i * s + j]       = m_angles_v[i].q[j];
            results[i * s + j + 6]   = m_angles_v[i].q_p[j];
            results[i * s + j + 12]  = m_angles_v[i].q_pp[j];
        }
    }
}

void Solver::MoveJ(const Angles &start_angle, const Angles &end_angle, const float &speed, const float &acceleration, const float &dt_millisec, vector<float> &results)
{
    scalar dt_s = ToSec(dt_millisec);
    Matrix4x4 buff[6];
    m_angles_v.resize(vector_size);
    counter max_size(0);
    counter min_size(0);
    counter sizes[6];
    counter size_of_arr(0);
    //        Angles pos;
    //        for (int i = 0; i < 6; i++)
    //            pos.q[i] = m_angles_v[0].q[i];
    CheckTime ct;
    m_angles_v[0] = start_angle;
    for (int i = 0; i < 6; i++)
    {
        size_of_arr = CalcTrajOneJoint(start_angle, end_angle, dt_s, speed, acceleration, i);
        sizes[i] = size_of_arr;
        if (max_size < size_of_arr)
            max_size = size_of_arr;
        if (min_size > size_of_arr)
            min_size = size_of_arr;
    }
    m_angles_v.resize(max_size);
    //        m_speed_v.resize(max_size);
    //        m_accel_v.resize(max_size);
    for (counter i = 0; i < 6; i++)
    {
        for (counter j = sizes[i] + 1; j < max_size; j++)
            m_angles_v[j].q[i] = m_angles_v[sizes[i]].q[i];
    }
    //Коллизии + динамика
    CalcCollisionsDynamics(max_size, dt_millisec);
    AnglesVectorToVector(max_size, results);
    //        memcpy(results.data(), m_angles_v.data(),  0 /*max_size*/ /** sizeof(Angles)*/);
    //        for (int i = 0; i < m_angles_v; i++)
    //        {

    //        }

    //#if !MULTITHREAD
    //        for (uint k = 0; k < max_size; k++)
    //        {
    //            SolveDirectKinematic(m_angles_v[k], buff);
    //            if (MoveMeshes(buff))
    //            {
    //#if WITH_COLLISIONS
    //                CheckCollision(&k);
    //#endif
    //            }
    //            SolveForwardDynamic(m_angles_v[k]);
    //        }
    //#endif
    //#if MULTITHREAD
    //        bool dynamic_calculated(0);
    //        bool collisions_calculated(0);
    //        while(1)
    //        {
    //            while (!exception_queue.empty())
    ////            try {
    //            {
    //                exception_ptr p = exception_queue.front();
    //                exception_queue.pop();
    //                rethrow_exception(p);
    //            }
    ////            }
    ////            }  catch (std::exception const &e) {
    ////                cout << "FAIL rethrowing exception" << e.what() << endl;
    ////            }
    //            if (m_moments_counter < max_size)
    //            {
    //                m_dynamic_notified = true;
    //                m_dynamic_cv.notify_one();
    //            } else {
    //                dynamic_calculated = true;
    //            }
    //            if (m_collisions_counter < max_size)
    //            {
    //                m_collision_notified = true;
    //                m_collision_cv.notify_one();
    //            } else {
    //                collisions_calculated = true;
    //            }
    //            if (dynamic_calculated && collisions_calculated)
    //                break;
    //        }
    //#endif

    VisualizeMoments(dt_millisec);
#if WITH_WINDOW
    m_emulator->MoveManipLinear(m_angles_v, m_speed_v, dt_millisec);
#endif
}

int Solver::CalcTrajOneJoint(const Angles &start_angle, const Angles &end_angle, const scalar &dt_s, const scalar &angle_max_speed, const scalar &angle_acceleration, const int &element_number)
{
    //Calculate trajectory for one joint and write result into array
    if (start_angle.q[element_number] == end_angle.q[element_number])
        return 0;
    scalar angle_pos = start_angle.q[element_number];
    scalar angle_accel(0);
    scalar angle_speed(0);
    scalar t_stop(0);
    counter ctr(0);
    double distance_next_step(0);
    bool breaking(0);
    scalar direction = (end_angle.q[element_number] - start_angle.q[element_number]) /
            abs(end_angle.q[element_number] - start_angle.q[element_number]);
    while (start_angle.q[element_number] != end_angle.q[element_number])
    {
        m_angles_v[ctr].q[element_number] = angle_pos;
        //            m_angles_v[ctr].q_p[element_number] = angle_max_speed;
        //            angle_speed += angle_acceleration * dt_s;
        distance_next_step = end_angle.q[element_number] - angle_pos;
        if (abs(distance_next_step) <= abs(abs(angle_speed) * t_stop - angle_acceleration * t_stop * t_stop * 0.5) && !breaking)
            breaking = true;
        if (breaking)
        {
            angle_accel = (2 * (distance_next_step - angle_speed * t_stop) * (1/t_stop/t_stop));
            angle_speed += angle_accel * dt_s;
        }
        else
        {
            if (abs(angle_speed + angle_accel * dt_s) >= abs(angle_max_speed)) {
                angle_speed = angle_max_speed * direction;
                angle_accel = 0;}
            else {
                angle_accel = angle_acceleration * direction;
                angle_speed += angle_accel * dt_s;}
        }
        t_stop = abs(angle_speed) / abs(angle_acceleration);
        // сделать отслеживание последней итерации
        angle_pos += angle_speed * dt_s + angle_accel * dt_s * dt_s / 2;
        m_angles_v[ctr].q_pp[element_number] = angle_accel;
        if (ctr + 1 >= m_angles_v.size())
            VectorsResize(ctr);
        ctr++;
        m_angles_v[ctr].q_p[element_number] = angle_speed;
        if (t_stop < dt_s)
            //last iter
        {
            m_angles_v[ctr].q[element_number] = end_angle.q[element_number];
            m_angles_v[ctr].q_p[element_number] = 0;
            break;
        }
    }
    return ctr;
}

void Solver::VectorsResize(const counter &size)
{
#if MULTITHREAD
    while(1)
    {
        unique_lock<mutex> lock_dynamic(m_dynamic_mtx);
        unique_lock<mutex> lock_collision(m_collision_mtx);
        cout << "VECTOR RESIZE from " << m_angles_v.size() << " to " << 2 * (size + 1) << endl;
        m_moments_v.resize(2 *  (size + 1));
        m_angles_v.resize(2 *   (size + 1));
        m_accel_v.resize(2 *    (size + 1));
        m_speed_v.resize(2 *    (size + 1));
        break;
    }
#else
    cout << "VECTOR RESIZE from " << m_angles_v.size() << " to " << 2 * (size + 1) << endl;
    m_moments_v.resize(2 * (size + 1));
    m_angles_v.resize(2  * (size + 1));
    m_accel_v.resize(2   * (size + 1));
    m_speed_v.resize(2   * (size + 1));
#endif
}

void Solver::CheckCollision(const uint *elem_number)
{
    uint k(0);
    if (elem_number == nullptr)
        k = m_collisions_counter;
    else
        k = *elem_number;
    for (uint j, i = 0; i < m_meshes_buff.size() - 1; i++)
    {
        if (m_meshes_buff[i].CheckCollision(m_meshes_buff[8]) ||
                [&](){
                for (j = i + 2; j < 8; j++)
                if (m_meshes_buff[i].CheckCollision(m_meshes_buff[j]))
                return true;
                return false;}())
        {
#if !WITH_BENCHMARK
            m_emulator->OneIterMoveLinear(m_angles_v[k], m_angles_v[k + 1],  DT_MILLISEC, false, m_accel_v[i]);
#endif
#if MULTITHREAD
            try {
                throw std::runtime_error("HOLY SHIT! COLLISION! Between " + to_string(i) + " and " + to_string(j)+ " elements");
            }  catch (...) {
                exception_ptr p = current_exception();
                exception_queue.push(p);
            }
#else
            throw std::runtime_error("HOLY SHIT! COLLISION! Between " + to_string(i) + " and " + to_string(j) + " elements");
#endif
        }
    }
}

bool Solver::MoveC(const Angles &start_angle, const StateVector &start_point, const StateVector &mid_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const scalar &dt_millisec, vector<float> &results)
{
    scalar dt_s = ToSec(dt_millisec);
    Vector3 A = start_point.GetCoord();
    Vector3 B = mid_point.GetCoord();
    Vector3 C = end_point.GetCoord();
    Vector3 AB = A - B;
    Vector3 BC = B - C;
    Vector3 AC = A - C;
    m_moments_v.resize(vector_size);
    m_angles_v.resize(vector_size);
    m_speed_v.resize(vector_size);
    m_accel_v.resize(vector_size);
    std::vector<Vector3> points;
    points.resize(vector_size);
    points[0] = A;

    //Проверка точек на нахождение на одной прямой
    //        auto lol = abs(0.5 * AB.TriangleS(AC));
    if (abs(0.5 * AB.TriangleS(AC)) < 10e-10)
        throw std::runtime_error ("Три точки лежат на одной прямой, измените точки");

    SolveInverseKinematic(start_point, start_angle, m_angles_v[0]);
    //____________________________________________
    Vector3 N = AB * BC;
    Vector3 AB2 = (A + B) / 2;
    Vector3 BC2 = (B + C) / 2;
    Vector3 ABn, BCn, O, R, N3, angle_range,
            Okr, v, a, a_ort, n, angles;
    scalar t, v_s, a_s; //t1,
    //Данная конструкция позволяет избежать вырождения решения в случае, когда нормаль
    //окружности не пересекает плоскость, образованную приравниванием одной из осей единице, например (1, y, z).
    Vector3 N1 = N * AB;
    Vector3 N2 = N * BC;
    N3 = N1 * N;
    Vector3 N4 = N3 * N1;
    N4 = N4 / N4.Length();
    scalar l;
    counter size(0);
    scalar t_stop;
    scalar distance_angle_to_stop;
    scalar end_angle, current_range,
            current_angle, start_angle_range;
    short direction(1);
    if ((N1.y * N2.x - N1.x * N2.y) != 0)
    {
        t =  (AB2.x * N2.y - AB2.y * N2.x - BC2.x * N2.y + BC2.y * N2.x) / (N1.y * N2.x - N1.x * N2.y);
//        t1 = (AB2.x * N1.y - AB2.y * N1.x - BC2.x * N1.y + BC2.y * N1.x) / (N1.y * N2.x - N1.x * N2.y);
    }
    else if ((N1.z * N2.x - N1.x * N2.z) != 0)
    {
        t  = (AB2.x * N2.z - AB2.z * N2.x - BC2.x * N2.z + BC2.z * N2.x) / (N1.z * N2.x - N1.x * N2.z);
//        t1 = (AB2.x * N1.z - AB2.z * N1.x - BC2.x * N1.z + BC2.z * N1.x) / (N1.z * N2.x - N1.x * N2.z);
    }
    else
    {
        if ((N1.z * N2.y - N1.y * N2.z) != 0)
            throw runtime_error("Normal to circle plane is NULL in moveC");
        t  = (AB2.y * N2.z - AB2.z * N2.y - BC2.y * N2.z + BC2.z * N2.y) / (N1.z * N2.y - N1.y * N2.z);
//        t1 = (AB2.y * N1.z - AB2.z * N1.y - BC2.y * N1.z + BC2.z * N1.y) / (N1.z * N2.y - N1.y * N2.z);
    }

    O = Vector3(N1.x * t + AB2.x, N1.y * t + AB2.y, N1.z * t + AB2.z);
    R = A - O;
    auto GetPoint = [&](const scalar &ang){return Vector3(N1/N1.Length() * R.Length() * sin(ang) + N3/N3.Length() * R.Length() * cos(ang) + O);};
    //Угол между начальной точкой и точкой P
    Vector3 P0;
    P0 = GetPoint(0);
    auto GetAngle = [&](const Vector3 &V){
        scalar angle = (V - O).Angle(P0 - O);
        if ((GetPoint(angle) - V).Length() > 10e-8)
            angle *= -1;
        return angle;};
    current_angle = GetAngle(A);
    if ((GetPoint(current_angle) - A).Length() < 10e-6)
        direction = 1;
    else
        direction = -1;
    bool breaking(0);
    angle_range = (start_point.GetAngles() - end_point.GetAngles());
    current_range = (C - A).Length();
    end_angle = GetAngle(end_point.GetCoord());
    start_angle_range = (end_point.GetCoord() - O).Angle(A - O);
    while(current_range > (v.Length() * dt_s - a_tcp * dt_s * dt_s / 2))
    {
        Vector3 point;
        if (breaking) //Режим торможения
        {
            a_ort = (points[size] - points[size - 1]).Normalize();
            a_s = /*-a_ort * a_tcp;//*/(2 * (distance_angle_to_stop - v.Length() * t_stop) /t_stop/t_stop);
            //                a_s = -a_ort * a_tcp;
            a = a_ort * a_tcp;
            current_angle += direction * (v.Length() * dt_s - a.Length() * dt_s * dt_s / 2) / R.Length();
            //                v += a * dt_s;
            v_s -= a_tcp * dt_s;
            v = a_ort * v_s;
            point = GetPoint(direction * current_angle);
        } else //Режим разгона
        {
            current_angle += direction * (v.Length() * dt_s + a_tcp * dt_s * dt_s / 2) / R.Length();
            point = GetPoint(direction * current_angle);
            //                current_angle = GetAngle(point);
            a_ort = (point - points[size]).Normalize();
            a = a_ort * a_tcp;
            v += a * dt_s;
            if (v.Length() > v_tcp)
                v = a_ort * v_tcp;
        }
        distance_angle_to_stop = (end_point.GetCoord() - O).Angle(point - O);
        angles = start_point.GetAngles() + angle_range * ((distance_angle_to_stop - start_angle_range) / start_angle_range);
        current_range = (C - point).Length();
        l = v.Length() * dt_s + a.Length() * dt_s * dt_s / 2;
        size++;
        points[size] = point;
        SolveInverseKinematic(StateVector(points[size], angles), m_angles_v[size - 1], m_angles_v[size]);
        t_stop = v.Length() / a_tcp;
        if ((distance_angle_to_stop <= (v.Length() * t_stop - a.Length() * t_stop * t_stop * 0.5) / R.Length()) && !breaking)
        {
            breaking = true;
            v_s = v.Length();
            //                break;
        }
        if (size > 1000)
            throw std::runtime_error("Cant solve MoveC task with 1000 iterations");
        if (size + 1 >= m_angles_v.size())
        {
            m_angles_v.resize(2 *(size + 1));
            points.resize(2 * (size + 1));
        }
        //                VectorsResize(size);
    }
    SolveInverseKinematic(end_point, m_angles_v[size], m_angles_v[size + 1]);
    m_angles_v.resize(size + 2);
    for (counter i = 0; i < size; i++)
    {
        for (uint j = 0; j < JOINTS_COUNT; j++)
        {
            m_angles_v[i].q_pp[j] = (m_angles_v[i + 1].q[j] - m_angles_v[i].q[j] - m_angles_v[i].q_p[j] * dt_s) / (dt_s * dt_s);
            m_angles_v[i + 1].q_p[j] = m_angles_v[i].q_p[j] + m_angles_v[i].q_pp[j] * dt_s;
        }
    }

    //        throw std::runtime_error("HOLY SHIT! COLLISION! Between " + to_string(3) + " and " + to_string(6)  /*to_string(cube)*/ + " elements");
    CalcCollisionsDynamics(size, dt_millisec);
    AnglesVectorToVector(size, results);
#if WITH_WINDOW
    m_emulator->moments_window.PlotCoordinates(m_moments.tao[0], m_moments.tao[1],m_moments.tao[2],m_moments.tao[3],m_moments.tao[4],m_moments.tao[5], dt_millisec, true);
    m_emulator->MoveManipLinear(m_angles_v, m_speed_v, dt_millisec);
#endif
    return true;
}

void Solver::CalcCollisionsDynamics(const scalar &array_size, const scalar &dt_millisec)
{
    Matrix4x4 buff[6];
    for (counter size = 0; size < array_size; size++)
    {
        SolveDirectKinematic(m_angles_v[size], buff);
#if WITH_COLLISIONS && !MULTITHREAD
        //Mesh calculations
        if (MoveMeshes(buff))
        {
            for (uint j, i = 0; i < m_meshes_buff.size() - 1; i++)
            {
                if (m_meshes_buff[i].CheckCollision(m_meshes_buff[8])||
                        [&](){
                        for (j = i + 2; j < 8; j++)
                        if (m_meshes_buff[i].CheckCollision(m_meshes_buff[j]))
                        return true;
                        return false;}())
                {
                    //show collision in visual
#if !WITH_BENCHMARK
                    m_emulator->OneIterMoveLinear(m_angles_v[size], m_angles_v[size + 1],  dt_millisec, false, m_accel_v[i]);
#endif
                    throw std::runtime_error("HOLY SHIT! COLLISION! Between " + to_string(i) + " and " + to_string(j)  /*to_string(cube)*/ + " elements");
                }
            }
        }
        else
            cout << "Can't calculate all mesh points for this configuration" << endl;
#endif
#if !MULTITHREAD
        SolveForwardDynamic(m_angles_v[size]);
        m_moments_v[size] = m_moments;
#endif
    }
#if MULTITHREAD && WITH_COLLISIONS
    bool dynamic_calculated(0);
    bool collisions_calculated(0);
    while(1)
    {
        while (!exception_queue.empty())
        {
            exception_ptr p = exception_queue.front();
            exception_queue.pop();
            rethrow_exception(p);
        }
        /*}  catch (std::exception const &e) {
                cout << "FAIL rethrowing exception" << e.what() << endl;
            }*/
        if (m_moments_counter < array_size)
        {
            m_dynamic_notified = true;
            m_dynamic_cv.notify_one();
        } else {
            dynamic_calculated = true;
        }
        if (m_collisions_counter < array_size)
        {
            m_collision_notified = true;
            m_collision_cv.notify_one();
        } else {
            collisions_calculated = true;
        }
        if (dynamic_calculated && collisions_calculated)
            break;
    }
#endif
}

scalar Solver::FindAngle(const Vector3 &O, const Vector3 &A0, const Vector3 P)
{
    Vector3 A0O;
    Vector3 PO;
    A0O = A0 - O;
    PO = P - O;
    return acos(A0O.Dot(PO) / (A0O.Length() * PO.Length()));
}
