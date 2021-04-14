#include "solver.h"


Solver::Solver(Robo &manip)
{
    manip_config = new Matrix(*manip.state_matrix);
    m_meshes.resize(9);
    m_mass.resize(6);
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
    m_inertion[0] = {0.000588, 0.000587, 0.000929, -1.299e-8, 2.44e-8, 1.79e-6};
    m_inertion[1] = {0.000928, 0.001512, 0.000937, 1.86e-5,  1.55e-10, 5.34e-12};
    m_inertion[2] = {0.001759, 0.000747, 0.001913, 2.32e-8,  6.37e-9,  1.68e-6};
    m_inertion[3] = {0.000198, 0.000479, 0.002059, 3.16e-11, 5.59e-10, 1.59e-7};
    m_inertion[4] = {9.826e-5, 0.000128, 0.000114, -2.74e-9, 4.41e-9,  5.82e-7};
    m_inertion[5] = {5.529e-5, 5.359e-5, 8.415e-5, 1.99e-10, 2.53e-9,  1.85e-7};
    m_mass[0] = 0.5982;
    m_mass[1] = 1.1495;
    m_mass[2] = 0.6399;
    m_mass[3] = 0.3551;
    m_mass[4] = 0.1747;
    m_mass[5] = 0.1667;
    m_cm[0] = Vector({0, 0, 0, 1});
    m_cm[1] = Vector({0, 0, 0, 1});
    m_cm[2] = Vector({0, 0, 0, 1});
    m_cm[3] = Vector({0, 0, 0, 1});
    m_cm[4] = Vector({0, 0, 0, 1});
    m_cm[5] = Vector({0, 0, 0, 1});

    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        m_ji[i] = {{(-m_inertion[i].xx() + m_inertion[i].yy() + m_inertion[i].zz())/2, m_inertion[i].xy(), m_inertion[i].xz(), m_mass[i] * m_cm[i](0)},
                   {m_inertion[i].xy(), (m_inertion[i].xx() - m_inertion[i].yy() + m_inertion[i].zz())/2,  m_inertion[i].yz(), m_mass[i] * m_cm[i](1)},
                   {m_inertion[i].xz(), m_inertion[i].yz(), (m_inertion[i].xx() + m_inertion[i].yy() - m_inertion[i].zz())/2,  m_mass[i] * m_cm[i](2)},
                   {m_mass[i] * m_cm[i](0),  m_mass[i] * m_cm[i](1), m_mass[i] * m_cm[i](2), m_mass[i]}};
//            m_ji[i].MatrixCout("m_ji");
    }
    m_moments_counter = 0;
    m_collisions_counter = 0;

#if !WITH_BENCHMARK
    m_emulator = new Emulator(manip);
#endif
    uint vector_size = 500;
    m_moments_vector.resize(vector_size);
    m_angles_v.resize(vector_size);
    m_speed_v.resize(vector_size);
    m_accel_v.resize(vector_size);
#if MULTITHREAD
    m_dynamic_thread = new thread(&Solver::SolveForwardDynamicMultithread, this);
    m_collision_thread = new thread(&Solver::CheckCollisionMultithread, this);
    m_dynamic_thread->detach();
    m_collision_thread->detach();
#endif
}
void Solver::SolveDirectKinematic(const Angles &q, Matrix4x4 TR[], Matrix4x4 *T)
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
                {                                0,                                 0,                 0,                             1}};
    }

    TR[0] = T[0];
    TR[1] = T[0] * T[1];
    TR[2] = T[0] * T[1] * T[2];
    TR[3] = T[0] * T[1] * T[2] * T[3];
    TR[4] = T[0] * T[1] * T[2] * T[3] * T[4];
    TR[5] = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
}

void Solver::SolveInverseKinematic(const scalar &accuracy, const StateVector &target, const Angles &curr_angle, Angles &res_angle)
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
//        aug_matrix.MatrixCout("a");
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
//            cout << "Точка x(" << target.x << ") y(" << target.y << ") z(" << target.z
//                 << ") roll(" << ToDeg(target.roll) << ") pitch(" << ToDeg(target.pitch)
//                 << ") yaw(" << ToDeg(target.yaw) << ") недостижима" << endl;
            QString exc_str;
            std::exception exc;
            std::string str;
            str = "Точка недостижима: x(" + to_string(target.x) + ") y(" + to_string(target.y) + ") z(" + to_string(target.z)
                    + ") roll(" + to_string(ToDeg(target.roll)) + ") pitch(" + to_string(ToDeg(target.pitch)) + ") yaw(" + to_string(ToDeg(target.yaw)) + ")";
//            exc_str.push_back("x(");  exc_str.push_back(QString::number(target.x));
//            exc_str.push_back(") y(");      exc_str.push_back(QString::number(target.y));
//            exc_str.push_back(") z(");      exc_str.push_back(QString::number(target.z));
//            exc_str.push_back(") roll(");   exc_str.push_back(QString::number(ToDeg(target.roll)));
//            exc_str.push_back(") pitch(");  exc_str.push_back(QString::number(ToDeg(target.pitch)));
//            exc_str.push_back(") yaw(");    exc_str.push_back(QString::number(ToDeg(target.yaw))); exc_str.push_back(")");
#if WITH_BENCHMARK
            cout << " cant move to this point" << endl;
#else
            throw str;
#endif
//            return -1;
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
//    return 0;
}

//int Solver::CalcPolynomMaxQPP(const StateVector &start_point, const StateVector &end_point, const Angles &curr_angles, Angles& res_angle, const int &t_millisec, const scalar &dt_millisec, Polynom result[])
//{
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
//return 0;
//}
void Solver::CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const Angles &curr_angles, Angles& res_angle, const ushort &dt_millisec, Polynom result[])
{
#define VECTOR
    uint vector_size = 500;
    m_moments_vector.resize(vector_size);
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

    SolveInverseKinematic(ACCURACY, start_point, curr_angles, m_angles_v[0]);
    SolveDirectKinematic(m_angles_v[0], buff);

    uint size(0);
    bool is_last(0);
    bool breaking(false);
    while (Distance(end_point, StateVector(x, y, z, rol, pit, yaw)) > 10e-5 && !is_last)
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
        auto distance_next_step = Distance(end_point, StateVector(x + vx *   dt_s + ax*dt_s*dt_s/2,
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
            ax = (2 * (end_point.x - x - vx * t_stop) * (1/t_stop/t_stop));// * abs(dist_x_norm);
            ay = (2 * (end_point.y - y - vy * t_stop) * (1/t_stop/t_stop));// * abs(dist_y_norm);
            az = (2 * (end_point.z - z - vz * t_stop) * (1/t_stop/t_stop));// * abs(dist_z_norm);
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

#if MULTITHREAD
        if (size + 1 >= m_angles_v.size())
        {
            while(1)
            {
                unique_lock<mutex> lock_dynamic(m_dynamic_mtx);
                unique_lock<mutex> lock_collision(m_collision_mtx);
                cout << "VECTORS RESIZED!!!!!! From " << m_angles_v.size() << " to " << 2 * (size + 1) << endl;
                m_moments_vector.resize(2 * (size + 1));
                m_angles_v.resize(2 * (size + 1));
                m_accel_v.resize(2 * (size + 1));
                m_speed_v.resize(2 * (size + 1));
                break;
            }
        }
#else
        if (size + 1 >= m_angles_v.size())
        {
            m_moments_vector.resize(2 * (size + 1));
            m_angles_v.resize(2 * (size + 1));
            m_accel_v.resize(2 * (size + 1));
            m_speed_v.resize(2 * (size + 1));
        }
#endif
        SolveInverseKinematic(ACCURACY, StateVector(x, y, z, rol, pit, yaw), m_angles_v[size], m_angles_v[size+1]);
        SolveDirectKinematic(m_angles_v[size + 1], buff);
//        distance = Distance(end_point, StateVector(x + vx * dt_s + ax*dt_s*dt_s/2, vy * dt_s + ay*dt_s*dt_s/2, vz * dt_s + az*dt_s*dt_s/2, rol + 0, pit + 0, yaw + 0));
        m_speed_v[size + 1].x = vx;
        m_speed_v[size + 1].y = vy;
        m_speed_v[size + 1].z = vz;
        m_accel_v[size].x = vx;
        m_accel_v[size].y = vy;
        m_accel_v[size].z = vz;
#if WITH_COLLISIONS && !MULTITHREAD
        //Mesh calculations
        {
            if (MoveMeshes(buff))
            {
                for (uint i = 0; i < m_meshes_buff.size() - 1; i++)
                {
                    if (/*meshes_buff[i].CheckCollision(meshes_buff[j]) || */m_meshes_buff[i].CheckCollision(m_meshes_buff[8]))
                    {
//                                                cout << "HOLY SHIT! COLLISION! Between " << i << " and " << j << " elements" << endl;
                        //show collision in visual
                    #if !WITH_BENCHMARK
                    #ifdef VECTOR
                        m_emulator->OneIterMoveLinear(m_angles_v[size], m_angles_v[size + 1],  dt_millisec, false, m_accel_v[i]);
                    #else
                        emulator->OneIterMoveLinear(angles[size], angles[size + 1],  dt_millisec, false, accel[i]);
                    #endif
                    #endif

                        throw std::runtime_error("HOLY SHIT! COLLISION! Between " + to_string(i) + " and " + "cube"  /*to_string(cube)*/ + " elements");
                    }
                }
            }
            else
                cout << "Can't calculate all mesh points for this configuration" << endl;
        }
#endif
        for (uint j = 0; j < JOINTS_COUNT; j++)
        {
            m_angles_v[size].q_pp[j] = (m_angles_v[size + 1].q[j] - m_angles_v[size].q[j] - m_angles_v[size].q_p[j] * dt_s) / (dt_s * dt_s) /*0.1*/;
            m_angles_v[size + 1].q_p[j] = m_angles_v[size].q_p[j] + m_angles_v[size].q_pp[j] * dt_s;
        }
#if !MULTITHREAD
        SolveForwardDynamic(m_angles_v[size]);
        for (int i = 0; i < 6; i++)
//            cout << "p = " << m_moments.p[i] << " dv = " << m_moments.dv[i] << " hikm = " << m_moments.hikm[i] << endl;
//        cout << endl;
        m_emulator->moments_window.PlotCoordinates(m_moments.tao[0], m_moments.tao[1],m_moments.tao[2],m_moments.tao[3],m_moments.tao[4],m_moments.tao[5], dt_millisec);

#endif
//        cout << endl;
//        cout << "angles vector element added " << size << endl << endl;
//        m_angles_v[size] = m_angles_v[size];
//        cout << " angle 1 = " << m_angles_v[size].q[0]
//             << " angle 2 = " << m_angles_v[size].q[1]
//             << " angle 3 = " << m_angles_v[size].q[2] << endl
//             << " angle 4 = " << m_angles_v[size].q[3]
//             << " angle 5 = " << m_angles_v[size].q[4]
//             << " angle 6 = " << m_angles_v[size].q[5] << endl;


        size++;
//        cout << " size = " << size << endl;
        t+= dt_s;
    }

    m_angles_v.resize(size);
    m_speed_v.resize(size);
    m_accel_v.resize(size);
#if MULTITHREAD
    bool dynamic_calculated(0);
    bool collisions_calculated(0);
    while(1)
    {
        while (!exception_queue.empty())
        try {
            exception_ptr p = exception_queue.front();
            exception_queue.pop();
            rethrow_exception(p);
        }  catch (std::exception const &e) {
            cout << "FAIL rethrowing exception" << e.what() << endl;
        }
        if (m_moments_counter < size)
        {
            m_dynamic_notified = true;
            m_dynamic_cv.notify_one();
        } else {
            dynamic_calculated = true;
        }
        if (m_collisions_counter < size)
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
    for (uint i = 0; i < size - 1/*t_millisec / dt*/; i++)
    {
//        for (uint j = 0; j < JOINTS_COUNT; j++)
//        {
//#ifdef VECTOR
//            angles_v[i].q_pp[j] = (angles_v[i + 1].q[j] - angles_v[i].q[j] - angles_v[i].q_p[j] * dt_s) / (dt_s * dt_s) /*0.1*/;
//            angles_v[i + 1].q_p[j] = angles_v[i].q_p[j] + angles_v[i].q_pp[j] * dt_s;
//#else
//            //Acceleration
//            angles[i].q_pp[j] =(angles[i + 1].q[j] - angles[i].q[j] - angles[i].q_p[j] * dt_s) / (dt_s * dt_s) /*0.1*/;
//            //Speed
//            angles[i + 1].q_p[j] = angles[i].q_p[j] + angles[i].q_pp[j] * dt_s;

//#endif
//        }
//#if !MULTITHREAD
//        SolveForwardDynamic(angles_v[i]);
//#else
//        m_dynamic_notified = true;
//        m_dynamic_cv.notify_one();
//#endif
//    this_thread::sleep_for(chrono::milliseconds(100));
    }

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
    m_emulator->moments_window.PlotCoordinates(m_moments.tao[0], m_moments.tao[1],m_moments.tao[2],m_moments.tao[3],m_moments.tao[4],m_moments.tao[5], dt_millisec, true);
    m_emulator->MoveManipLinear(m_angles_v, m_accel_v, dt_millisec);
    res_angle = m_angles_v.back();
//    m_emulator->moments_window.PlotCoordinates(m_moments.tao[0], m_moments.tao[1],m_moments.tao[2],m_moments.tao[3],m_moments.tao[4],m_moments.tao[5], dt_millisec,true);
#endif

//    cout << "CalcPolynomLine ended" << endl;

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
//        throw QString("cant reach point") + QString::number(distance);
        return false;
    }
    return true;
}

bool Solver::MoveMeshes(const Matrix4x4 TR[])
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

void Solver::CalcUMatrix()
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
                    U[k*6 + j].SelfMultipl(m_T[ctr], cq);// = U[k*6 + j] * T[ctr] * q;
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
    auto q = &m_angles_v[m_moments_counter];
    while(1)
    {
        while(!m_dynamic_notified)
        {
            m_dynamic_cv.wait(lock);
        }
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
                        U[k*6 + j].SelfMultipl(T[ctr], cq);// = U[k*6 + j] * T[ctr] * q;
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
        for ( int i = 0; i < 6; i++)
        {
            m_moments.tao[i] = m_moments.p[i] + m_moments.dv[i] + m_moments.hikm[i];
        }
        m_moments_vector[m_moments_counter] = m_moments;
        for (int i = 0; i < 6; i++)
//            cout << "p = " << m_moments.p[i] << " dv = " << m_moments.dv[i] << " hikm = " << m_moments.hikm[i] << endl;
//        cout << endl;

//        cout << "size of moments vec tor = " << m_moments_counter << endl;
        m_emulator->moments_window.PlotCoordinates(m_moments_vector[m_moments_counter].tao[0], m_moments_vector[m_moments_counter].tao[1],
                                                   m_moments_vector[m_moments_counter].tao[2],m_moments_vector[m_moments_counter].tao[3],
                                                   m_moments_vector[m_moments_counter].tao[4],m_moments_vector[m_moments_counter].tao[5], dt_millisec);
        m_moments_counter++;
        m_dynamic_notified = false;
        m_dynamic_cv.wait(lock);
    }
#endif
}

void Solver::CheckCollisionMultithread()
{
#if MULTITHREAD
    Matrix4x4 buff1[6];
    unique_lock<mutex> lock(m_collision_mtx);
    Matrix4x4 T[6];
    float dt_millisec = 10;
    while(1)
    {
        while(!m_collision_notified)
        {
            m_collision_cv.wait(lock);
        }
        SolveDirectKinematic(m_angles_v[m_collisions_counter], buff1, T);

//        cout << m_angles_v[m_collisions_counter].q[0] << " "
//             << m_angles_v[m_collisions_counter].q[1] << " "
//             << m_angles_v[m_collisions_counter].q[2] << " "
//             << endl;

//        cout << " x = " << buff1[5].ConvertToVector3().x
//             << " y = " << buff1[5].ConvertToVector3().y
//             << " z = " << buff1[5].ConvertToVector3().z
//             << endl;
        if (MoveMeshes(buff1))
        {
            for (uint i = 0; i < m_meshes_buff.size() - 1; i++)
            {
                if (m_meshes_buff[i].CheckCollision(m_meshes_buff[8]) ||
                        [&](){
                        for (int j = i + 3; j < 6; j++)
                            if (m_meshes_buff[i].CheckCollision(m_meshes_buff[j]))
                                return true;
                        return false;}())

                {
                #if !WITH_BENCHMARK
                    m_emulator->OneIterMoveLinear(m_angles_v[m_collisions_counter], m_angles_v[m_collisions_counter + 1],  dt_millisec, false, m_accel_v[i]);
                #endif
                    try {
                        throw string("HOLY SHIT! COLLISION! Between " + to_string(i) + " and " + "cube"  /*to_string(cube)*/ + " elements");
                        cout << "HOLY SHIT! COLLISION! Between " + to_string(i) + " and " + "cube"  /*to_string(cube)*/ + " elements" << endl;
                    }  catch (...) {
                        exception_ptr p = current_exception();
                        exception_queue.push(p);
                    }
                }
            }

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
            //            U[i*6 + j].MatrixCout("U");
            //            m_cm[j].VectorCout("m_cm");
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
                    buff += Trace(u_buff * m_ji[i].MultiplyTranspose(U[i*6 + j])) * angles.q_p[i] * angles.q_p[i];
                }
                m_moments.hikm[i] += buff;
            }
        }
    }
    for ( int i = 0; i < 6; i++)
    {
        m_moments.tao[i] = m_moments.p[i] + m_moments.dv[i] + m_moments.hikm[i];
    }
#if MULTITHREAD
    m_moments_vector[m_moments_counter] = m_moments;
#endif

    //        cout << "tao 1 = " << m_moments.tao[0] << "\ntao 2 = " << m_moments.tao[1] << "\ntao 3 = " << m_moments.tao[2] <<
    //        "\ntao 4 = " << m_moments.tao[3] << "\ntao 5 = " << m_moments.tao[4] << "\ntao 6 = " << m_moments.tao[5] << endl << endl;

#if MULTITHREAD
//    cout << "Solve Forward Dynamic finished" << endl;
#endif
    return true;
}
