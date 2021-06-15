#include "solver.h"


Solver::Solver(Robo &robot)
{
    manip_config = new Matrix(*robot.state_matrix);
    m_meshes.resize(9);
    m_mass.resize(6);
    m_main_IKData = new IKData;

    try {
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
    }  catch (std::string &exc) {
        cout << exc << endl;
    }
//    m_inertion[0] = {0.000588, 0.000587, 0.000929, -1.299e-8, 2.44e-8, 1.79e-6};
//    m_inertion[1] = {0.000928, 0.001512, 0.000937, 1.86e-5,  1.55e-10, 5.34e-12};
//    m_inertion[2] = {0.001759, 0.000747, 0.001913, 2.32e-8,  6.37e-9,  1.68e-6};
//    m_inertion[3] = {0.000198, 0.000479, 0.002059, 3.16e-11, 5.59e-10, 1.59e-7};
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
    m_dynamic_IKData = new IKData;
    m_dynamic_thread = new thread(&Solver::SolveForwardDynamicMultithread, this);
    m_collision_thread = new thread(&Solver::CheckCollisionMultithread, this);
//    m_dynamic_thread->join();
//    m_collision_thread->join();
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
            std::exception exc;
            std::string str;
            str = "Точка недостижима: x(" + to_string(target.x) + ") y(" + to_string(target.y) + ") z(" + to_string(target.z)
                    + ") roll(" + to_string(ToDeg(target.roll)) + ") pitch(" + to_string(ToDeg(target.pitch )) + ") yaw(" + to_string(ToDeg(target.yaw)) + ")";
#if WITH_BENCHMARK
            cout << " cant move to this point" << endl;
#else
            throw std::runtime_error("Точка недостижима: x(" + to_string(target.x) + ") y(" + to_string(target.y) +
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
                   /*Angles& res_angle,*/ const ushort &dt_millisec, vector<float> results)
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
//        SolveDirectKinematic(m_angles_v[size + 1], buff);
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
            m_angles_v[i].q_pp[j] = (m_angles_v[i + 1].q[j] - m_angles_v[i].q[j] - m_angles_v[i].q_p[j] * dt_s) / (dt_s * dt_s) /*0.1*/;
            m_angles_v[i + 1].q_p[j] = m_angles_v[i].q_p[j] + m_angles_v[i].q_pp[j] * dt_s;
        }
    }
    CalcCollisionsDynamics(size, dt_millisec);
    m_angles_v.resize(size + 1);
    m_speed_v.resize(size + 1);
    m_accel_v.resize(size + 1);
//#if WITH_WINDOW
//    for (counter i = 0; i < size; i++)
//    {
//        m_emulator->moments_window.PlotCoordinates(m_moments_vector[i].tao[0], m_moments_vector[i].tao[1], m_moments_vector[i].tao[2],
//                m_moments_vector[i].tao[3],m_moments_vector[i].tao[4],m_moments_vector[i].tao[5], dt_millisec);
//    }
//#endif
    for (int j = 1; j < JOINTS_COUNT; j++)
    {
        m_angles_v[size].q_pp[j] = (m_angles_v[size].q[j] - m_angles_v[size - 1].q[j] - m_angles_v[size - 1].q_p[j] * dt_s) / (dt_s * dt_s) /*0.1*/;
        m_angles_v[size].q_p[j] = 0;
    }
    for (uint i = 0; i < size/*t_millisec / dt*/; i++)
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
                    U[k*6 + j] = U[k*6 + j] * m_T[ctr] * cq;//.SelfMultipl(m_T[ctr], cq);// = U[k*6 + j] * m_T[ctr] * cq;
                else
                {
                    U[k*6 + j] = U[k*6 + j] * m_T[ctr];
                }
            }
        }
    }
}

void Solver::SolveForwardDynamicMultithread()
{
#if MULTITHREAD
//    cout << "Solve Forward Dynamic started" << endl;
    unique_lock<mutex> lock(m_dynamic_mtx);
    int dt_millisec = 10;
    Matrix4x4 T[6];
    Matrix4x4 U[36];
    const Matrix &A = *manip_config;
    RoboMath::Angles *q;
    while(1)
    {
        while(!m_dynamic_notified)
        {
            m_dynamic_cv.wait(lock);
        }
        q = &m_angles_v[m_moments_counter];
//        cout << " a1 = " << m_angles_v[m_moments_counter].q[0]
//             << " a2 = " << m_angles_v[m_moments_counter].q[1]
//             << " a3 = " << m_angles_v[m_moments_counter].q[2] << endl
//             << " a4 = " << m_angles_v[m_moments_counter].q[3]
//             << " a5 = " << m_angles_v[m_moments_counter].q[4]
//             << " a6 = " << m_angles_v[m_moments_counter].q[5] << endl;
        T[0] = {{         cos(q->q[0]),         -sin(q->q[0]),       0,           A.at(0,0)},
                {sin(q->q[0]) * cos(0), cos(q->q[0]) * cos(0), -sin(0), -sin(0) * A.at(0,2)},
                {sin(q->q[0]) * sin(0), cos(q->q[0]) * sin(0),  cos(0),  cos(0) * A.at(0,2)},
                {                   0,                    0,       0,                   1}};        
        for (counter i = 1; i < JOINTS_COUNT; i++)
        {
            T[i] = {{cos(q->q[i]),                                      -sin(q->q[i]),                 0,                     A.at(i-1,0)},
                    {sin(q->q[i]) * cos(A.at(i-1,1)), cos(q->q[i]) * cos(A.at(i-1,1)), -sin(A.at(i-1,1)), -sin(A.at(i-1,1)) * A.at(i,2)},
                    {sin(q->q[i]) * sin(A.at(i-1,1)), cos(q->q[i]) * sin(A.at(i-1,1)),  cos(A.at(i-1,1)),  cos(A.at(i-1,1)) * A.at(i,2)},
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
                        U[k*6 + j] = U[k*6 + j] * T[ctr] * cq;// = U[k*6 + j] * T[ctr] * q;
                    else
                    {
                        U[k*6 + j] = U[k*6 + j] * T[ctr];
                    }
                }
            }
        }
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
            for (counter k = i; k < 6; k++)
            {
                buff = 0;
                for (counter j = RoboMath::max(i, k); j < 6; j++)
                {
                    buff += Trace(U[k*6 + j] * m_ji[k].MultiplyTranspose(U[i*6 + j]));
                }
                buff *= m_angles_v[m_moments_counter].q_pp[k];
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
                                u_buff = u_buff * cq_sq * T[ctr];
                            else if (ctr == k || ctr == m)
                                u_buff = u_buff * cq * T[ctr];
                            else
                                u_buff = u_buff * T[ctr];
                        }
                        buff += Trace(u_buff * m_ji[i].MultiplyTranspose(U[i*6 + j])) * m_angles_v[m_moments_counter].q_p[i] * m_angles_v[m_moments_counter].q_p[i];
                    }
                    m_moments.hikm[i] += buff;
                }
            }
        }
        for (int i = 0; i < 6; i++)
        {
            m_moments.tao[i] = m_moments.p[i] + m_moments.dv[i] + m_moments.hikm[i];
        }
        m_moments_vector[m_moments_counter] = m_moments;
//        for (int i = 0; i < 6; i++)
//            cout << "p = " << m_moments.p[i] << " dv = " << m_moments.dv[i] << " hikm = " << m_moments.hikm[i] << endl;
//        cout << endl;

//        cout << "size of moments vec tor = " << m_moments_counter << endl;
#if WITH_WINDOW
//        m_emulator->moments_window.PlotCoordinates(m_moments_vector[m_moments_counter].tao[0], m_moments_vector[m_moments_counter].tao[1],
//                                                   m_moments_vector[m_moments_counter].tao[2], m_moments_vector[m_moments_counter].tao[3],
//                                                   m_moments_vector[m_moments_counter].tao[4], m_moments_vector[m_moments_counter].tao[5], dt_millisec);
#endif
        m_moments_counter++;
//        cout << "moments counter " << m_moments_counter << endl;
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
    while(1)
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
//    cout << " counter " << m_moments_counter << endl;
    return true;
}

void Solver::VectorsResize(const counter &size)
{
#if MULTITHREAD
    while(1)
    {
        unique_lock<mutex> lock_dynamic(m_dynamic_mtx);
        unique_lock<mutex> lock_collision(m_collision_mtx);
        cout << "VECTOR RESIZE from " << m_angles_v.size() << " to " << 2 * (ctr + 1) << endl;
        m_moments_vector.resize(2 * (ctr + 1));
        m_angles_v.resize(2 * (ctr + 1));
        m_accel_v.resize(2 * (ctr + 1));
        m_speed_v.resize(2 * (ctr + 1));
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
