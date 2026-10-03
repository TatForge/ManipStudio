#include "solver.h"

namespace {

// Time-optimal trapezoidal (triangular for short moves) speed profile for a
// move of length `distance` under the limits v_max / a_max, stretched in time
// so that it ends exactly on a sample (n * dt). Stretching only lowers the
// speed and acceleration, so the limits always hold.
struct MotionProfile
{
    MotionProfile(scalar distance, scalar v_max, scalar a_max, scalar dt)
    {
        d = distance;
        if (d <= 0 || v_max <= 0 || a_max <= 0)
        {
            n = 0;
            return;
        }
        if (d >= v_max * v_max / a_max)
        {
            ta = v_max / a_max;
            tc = (d - v_max * v_max / a_max) / v_max;
        }
        else
        {
            ta = sqrt(d / a_max);
            tc = 0;
        }
        const scalar t_min = 2 * ta + tc;
        n = int(ceil(t_min / dt - 1e-9));
        if (n < 1)
            n = 1;
        const scalar k = t_min / (n * dt);
        ta /= k;
        tc /= k;
        T = n * dt;
        a = d / (ta * (ta + tc));
        v = a * ta;
    }
    scalar Pos(scalar t) const
    {
        if (n == 0 || t >= T) return d;
        if (t <= 0) return 0;
        if (t < ta) return 0.5 * a * t * t;
        if (t < ta + tc) return 0.5 * a * ta * ta + v * (t - ta);
        return d - 0.5 * a * (T - t) * (T - t);
    }
    scalar Vel(scalar t) const
    {
        if (n == 0 || t <= 0 || t >= T) return 0;
        if (t < ta) return a * t;
        if (t < ta + tc) return v;
        return a * (T - t);
    }
    scalar Acc(scalar t) const
    {
        if (n == 0 || t < 0 || t >= T) return 0;
        if (t < ta) return a;
        if (t < ta + tc) return 0;
        return -a;
    }
    scalar d = 0, v = 0, a = 0, ta = 0, tc = 0, T = 0;
    int n = 0; // number of dt steps
};

// Joint velocities / accelerations of a sampled joint path from its positions.
// q_p[k] is the backward difference, q_pp[k] the acceleration over [k, k+1].
void DifferentiateJointPath(vector<Angles> &path, int last, scalar dt)
{
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        path[0].q_p[j] = 0;
        for (int k = 1; k <= last; k++)
            path[k].q_p[j] = (path[k].q[j] - path[k - 1].q[j]) / dt;
        for (int k = 0; k < last; k++)
            path[k].q_pp[j] = (path[k + 1].q_p[j] - path[k].q_p[j]) / dt;
        path[last].q_pp[j] = 0;
    }
}

} // namespace

BoxAABB::BoxAABB(const scalar &x_min, const scalar &y_min, const scalar &z_min, const scalar &x_max, const scalar &y_max, const scalar &z_max)
    : point_max(x_max, y_max, z_max), point_min(x_min, y_min, z_min)
{
}

bool Solver::CheckAABB(const BoxAABB &box1, const BoxAABB &box2) const
{
    return box1.point_min.x <= box2.point_max.x && box1.point_max.x >= box2.point_min.x &&
           box1.point_min.y <= box2.point_max.y && box1.point_max.y >= box2.point_min.y &&
           box1.point_min.z <= box2.point_max.z && box1.point_max.z >= box2.point_min.z;
}


Solver::Solver(Robo &robot)
{
    manip_config = new Matrix(*robot.state_matrix);
    max_joint_moments = robot.GetMaxJointMoments();
    m_joint_limits = robot.GetJointLimits();
    m_floor_z = robot.GetFloorHeight();
    m_mass.resize(6);
    m_main_IKData = new IKData;
    //collision model: convex hulls of the arm shape (body frames), the obstacle last (world)
    m_parts = ArmShape::Build(*manip_config);
    for (const ArmPart &part : m_parts)
        m_meshes.emplace_back(ArmShape::HullPoints(part));
    MeshCollider obstacle;
    obstacle.ReadStl("STLmodels/cube0.5m.stl");
    obstacle.MoveMesh(Quaternion(0, 0, 0, 1), Vector3(SMESH + 0.05, SMESH - 0.05, 0));
    m_meshes.push_back(obstacle);
    const int obstacle_index = int(m_parts.size());
    for (int i = 0; i < obstacle_index; i++)
    {
        for (int j = i + 1; j < obstacle_index; j++)
            if (!ArmShape::Adjacent(m_parts[i].body, m_parts[j].body))
                m_check_pairs.emplace_back(i, j);
        m_check_pairs.emplace_back(i, obstacle_index);
    }
    m_meshes_buff = m_meshes;
    m_bounds.assign(m_meshes.size(), BoxAABB(0, 0, 0, 0, 0, 0));
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
        {
            data->d_thet.q[i] += data->delta_thetas(i);
            //keep the solution inside the joint limits; a target that needs more is unreachable
            data->d_thet.q[i] = std::min(std::max(data->d_thet.q[i], m_joint_limits[i].first), m_joint_limits[i].second);
        }
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
                   const ushort &dt_millisec, vector<float> &results)
{
    const scalar dt_s = ToSec(dt_millisec);
    m_moments_counter = 0;
    m_collisions_counter = 0;
    Vector3 p0 = start_point.GetCoord();
    Vector3 p1 = end_point.GetCoord();
    Vector3 ang0 = start_point.GetAngles();
    Vector3 d_ang = end_point.GetAngles() - ang0;
    const scalar distance = (p1 - p0).Length();
    Vector3 dir = distance > 0 ? (p1 - p0) / distance : Vector3(0, 0, 0);
    //A pure reorientation is timed along the largest Euler angle change, with
    //v_tcp / a_tcp taken as rad/s and rad/s^2
    const scalar path = distance > 0 ? distance : std::max({abs(d_ang.x), abs(d_ang.y), abs(d_ang.z)});
    if (path <= 0)
        throw std::runtime_error("MoveL: start and end poses are equal");
    MotionProfile profile(path, v_tcp, a_tcp, dt_s);
    const int last = profile.n;

    m_angles_v.assign(last + 2, Angles());
    m_moments_v.assign(last + 2, Robo::Moments());
    m_speed_v.assign(last + 2, Vector3());
    m_accel_v.assign(last + 2, Vector3());

    SolveInverseKinematic(start_point, curr_angles, m_angles_v[0]);
    for (int k = 1; k <= last; k++)
    {
        const scalar t = k * dt_s;
        const scalar u = profile.Pos(t) / path; //0..1
        StateVector target(p0 + dir * (u * distance), ang0 + d_ang * u);
        SolveInverseKinematic(target, m_angles_v[k - 1], m_angles_v[k]);
        m_speed_v[k] = dir * profile.Vel(t);
        m_accel_v[k] = dir * profile.Acc(t);
    }
    DifferentiateJointPath(m_angles_v, last, dt_s);
    m_angles_v[last + 1] = m_angles_v[last];

    CalcCollisionsDynamics(last + 1, dt_millisec);
    if (distance > 0)
        for (int i = 0; i <= last; i++)
            CheckTrajectoryForLinear(m_angles_v[i], p0, p1, DELTA_RANGE);
    AnglesVectorToVector(last + 1, results);
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
    Quaternion rotation[JOINTS_COUNT];
    Vector3 origin[JOINTS_COUNT];
    for (int k = 0; k < JOINTS_COUNT; k++)
    {
        rotation[k] = TR[k].ConvertToQuaternion();
        origin[k] = TR[k].ConvertToVector3();
    }
    m_meshes_buff = m_meshes;
    for (size_t i = 0; i < m_meshes_buff.size(); i++)
    {
        //the base and the obstacle stay where they are
        if (i < m_parts.size() && m_parts[i].body >= 0)
            m_meshes_buff[i].MoveMesh(rotation[m_parts[i].body], origin[m_parts[i].body]);
        const MeshCollider &m = m_meshes_buff[i];
        Vector3 lo(1e9, 1e9, 1e9), hi(-1e9, -1e9, -1e9);
        for (ushort k = 0; k < m.Size(); k++)
        {
            const Vector3 p = m.point(k);
            lo = Vector3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
            hi = Vector3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
        }
        m_bounds[i] = BoxAABB(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);
    }
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
            throw std::runtime_error("Calculated moment of the " + to_string(joint_number) + "'s engine (" + to_string(m_moments.tao[joint_number]) + ")is greated than the maximum possible[" + to_string(max_joint_moments[joint_number]) + "]");
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
    //Lagrange-Euler: tao = D(q) q'' + h(q, q') + G(q), with U(k,j) = dT0j/dq_k
    CalcUMatrix(angles);
    m_moments.MakeNull();
    scalar buff(0);
    //Gravity
    for (counter i = 0; i < 6; i++)
    {
        for (counter j = i; j < 6; j++)
        {
            m_moments.p[i] += - m_mass[j] * m_gv.MultiplyTranspose(U[i*6 + j].MultiplyTranspose(m_cm[j]));
        }
    }
    //Inertia: D(i,k) = sum_j Tr(U(k,j) J_j U(i,j)^T)
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = 0; k < 6; k++)
        {
            buff = 0;
            for (counter j = RoboMath::max(i, k); j < 6; j++)
            {
                buff += Trace(U[k*6 + j] * m_ji[j].MultiplyTranspose(U[i*6 + j]));
            }
            m_moments.dv[i] += buff * angles.q_pp[k];
        }
    }
    //Coriolis and centrifugal: h(i) = sum_k,m sum_j Tr(U(k,m,j) J_j U(i,j)^T) q'_k q'_m,
    //U(k,m,j) = d2T0j/dq_k dq_m (modified DH: dT_c/dq_c = T_c * Q)
    Matrix4x4 u_buff;
    for (counter i = 0; i < 6; i++)
    {
        for (counter k = 0; k < 6; k++)
        {
            for (counter m = 0; m < 6; m++)
            {
                if (angles.q_p[k] == 0 || angles.q_p[m] == 0)
                    continue;
                buff = 0;
                for (counter j = RoboMath::max(i, RoboMath::max(k, m)); j < 6; j++)
                {
                    u_buff.MakeIdentity();
                    for (counter ctr = 0; ctr <= j; ctr++)
                    {
                        u_buff = u_buff * m_T[ctr];
                        if (ctr == k)
                            u_buff = u_buff * cq;
                        if (ctr == m)
                            u_buff = u_buff * cq;
                    }
                    buff += Trace(u_buff * m_ji[j].MultiplyTranspose(U[i*6 + j]));
                }
                m_moments.hikm[i] += buff * angles.q_p[k] * angles.q_p[m];
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

Angles Solver::VectorToAngle(const vector<float> &v)
{
    Angles temp;
    for (uint8_t i = 0; i < JOINTS_COUNT; i++)
    {
        temp.q[i] = v[i];
        temp.q_p[i] = v[i + 6];
        temp.q_pp[i] = v[i + 12];
    }
    return temp;
}

void Solver::MoveJ(const Angles &start_angle, const Angles &end_angle, const float &speed, const float &acceleration, const float &dt_millisec, vector<float> &results)
{
    if (end_angle == start_angle)
        throw(std::runtime_error("cant move if position of all joints is equal"));
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        for (const Angles *a : {&start_angle, &end_angle})
        {
            if (a->q[i] < m_joint_limits[i].first || a->q[i] > m_joint_limits[i].second)
                throw std::runtime_error("MoveJ: joint " + to_string(i) + " " + (a == &start_angle ? "start" : "end") + " angle " +
                                         to_string(ToDeg(a->q[i])) + " deg is outside its limit [" + to_string(ToDeg(m_joint_limits[i].first)) +
                                         ", " + to_string(ToDeg(m_joint_limits[i].second)) + "]");
        }
    }
    const scalar dt_s = ToSec(dt_millisec);
    m_moments_counter = 0;
    m_collisions_counter = 0;
    //Synchronous PTP move: the joint with the largest travel gets the
    //time-optimal profile, all other joints follow the same normalised profile
    //scaled to their own travel, so every joint starts and stops together and
    //the path is a straight line in joint space. All joints share the same
    //limits, so the scaled joints stay below them.
    scalar max_delta = 0;
    for (int i = 0; i < JOINTS_COUNT; i++)
        max_delta = std::max(max_delta, scalar(abs(end_angle.q[i] - start_angle.q[i])));
    MotionProfile profile(max_delta, speed, acceleration, dt_s);
    const int last = profile.n;
    m_angles_v.assign(last + 2, Angles());
    m_moments_v.assign(last + 2, Robo::Moments());
    m_speed_v.assign(last + 2, Vector3());
    m_accel_v.assign(last + 2, Vector3());
    for (int k = 0; k < last + 2; k++)
    {
        const scalar t = k * dt_s;
        const scalar pos = profile.Pos(t) / max_delta; //0..1
        const scalar vel = profile.Vel(t) / max_delta;
        const scalar acc = profile.Acc(t) / max_delta;
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            const scalar delta = end_angle.q[i] - start_angle.q[i];
            m_angles_v[k].q[i]    = start_angle.q[i] + delta * pos;
            m_angles_v[k].q_p[i]  = delta * vel;
            m_angles_v[k].q_pp[i] = delta * acc;
        }
    }
    //Коллизии + динамика
    CalcCollisionsDynamics(last + 1, dt_millisec);
    AnglesVectorToVector(last + 1, results);
}

int Solver::CalcTrajOneJoint(const Angles &start_angle, const Angles &end_angle, const scalar &dt_s, const scalar &angle_max_speed, const scalar &angle_acceleration, const int &element_number)
{
    //Writes the trajectory of one joint into m_angles_v (holding the end angle
    //after it arrives) and returns the number of dt steps it needs
    const int j = element_number;
    const scalar delta = end_angle.q[j] - start_angle.q[j];
    const scalar direction = delta < 0 ? -1 : 1;
    MotionProfile profile(abs(delta), angle_max_speed, angle_acceleration, dt_s);
    if (m_angles_v.size() < size_t(profile.n + 1))
        m_angles_v.resize(profile.n + 1);
    for (size_t k = 0; k < m_angles_v.size(); k++)
    {
        const scalar t = k * dt_s;
        m_angles_v[k].q[j]    = start_angle.q[j] + direction * profile.Pos(t);
        m_angles_v[k].q_p[j]  = direction * profile.Vel(t);
        m_angles_v[k].q_pp[j] = direction * profile.Acc(t);
    }
    return profile.n;
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
    const uint k = elem_number == nullptr ? m_collisions_counter : *elem_number;
#if MULTITHREAD
    try {
        ThrowOnCollision(k, DT_MILLISEC);
    }  catch (...) {
        exception_queue.push(current_exception());
    }
#else
    ThrowOnCollision(k, DT_MILLISEC);
#endif
}

pair<int, int> Solver::FindCollision() const
{
    for (const auto &p : m_check_pairs)
    {
        if (!CheckAABB(m_bounds[p.first], m_bounds[p.second]))
            continue;
        //CheckCollision is not const; the meshes are copied per sample anyway
        MeshCollider a = m_meshes_buff[p.first];
        if (a.CheckCollision(m_meshes_buff[p.second]))
            return p;
    }
    return {-1, -1};
}

string Solver::MeshName(int index) const
{
    return index < int(m_parts.size()) ? m_parts[index].name : "obstacle";
}

void Solver::ThrowOnCollision(uint sample, scalar dt_millisec)
{
    const pair<int, int> hit = FindCollision();
    const int below = hit.first < 0 ? FindMeshBelowFloor() : -1;
    if (hit.first < 0 && below < 0)
        return;
#if !WITH_BENCHMARK
    //show the pose in the visualizer
    if (sample + 1 < m_angles_v.size() && sample < m_accel_v.size())
        m_emulator->OneIterMoveLinear(m_angles_v[sample], m_angles_v[sample + 1], dt_millisec, false, m_accel_v[sample]);
#endif
    if (hit.first >= 0)
        throw std::runtime_error("HOLY SHIT! COLLISION! Between " + MeshName(hit.first) + " and " + MeshName(hit.second));
    throw std::runtime_error("HOLY SHIT! COLLISION! Element " + MeshName(below) + " is below the floor (z = " + to_string(m_floor_z) + ")");
}

bool Solver::MoveC(const Angles &start_angle, const StateVector &start_point, const StateVector &mid_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const scalar &dt_millisec, vector<float> &results)
{
    const scalar dt_s = ToSec(dt_millisec);
    m_moments_counter = 0;
    m_collisions_counter = 0;
    Vector3 A = start_point.GetCoord();
    Vector3 B = mid_point.GetCoord();
    Vector3 C = end_point.GetCoord();
    Vector3 AB = A - B;
    Vector3 AC = A - C;
    //Проверка точек на нахождение на одной прямой
    if (abs(0.5 * AB.TriangleS(AC)) < 10e-10)
        throw std::runtime_error ("Три точки лежат на одной прямой, измените точки");

    //Circle through A, B, C
    Vector3 ab = B - A;
    Vector3 ac = C - A;
    Vector3 n = ab * ac;
    Vector3 O = A + ((n * ab) * ac.Dot(ac) + (ac * n) * ab.Dot(ab)) / (2 * n.Dot(n));
    const scalar R = (A - O).Length();
    //Travelling A -> B -> C is counter-clockwise around (B - A) x (C - B)
    Vector3 axis = ((B - A) * (C - B)).Normalize();
    Vector3 u = (A - O) / R;
    Vector3 w = axis * u;
    auto angle_of = [&](const Vector3 &P) {
        Vector3 r = P - O;
        scalar angle = atan2(r.Dot(w), r.Dot(u));
        return angle < 0 ? angle + 2 * M_PI : angle;
    };
    const scalar sweep = angle_of(C);
    const scalar arc_length = R * sweep;
    Vector3 ang0 = start_point.GetAngles();
    Vector3 d_ang = end_point.GetAngles() - ang0;
    MotionProfile profile(arc_length, v_tcp, a_tcp, dt_s);
    const int last = profile.n;

    m_angles_v.assign(last + 2, Angles());
    m_moments_v.assign(last + 2, Robo::Moments());
    m_speed_v.assign(last + 2, Vector3());
    m_accel_v.assign(last + 2, Vector3());

    SolveInverseKinematic(start_point, start_angle, m_angles_v[0]);
    for (int k = 1; k <= last; k++)
    {
        const scalar t = k * dt_s;
        const scalar s = profile.Pos(t);
        const scalar phi = s / R;
        Vector3 point = O + (u * cos(phi) + w * sin(phi)) * R;
        StateVector target(point, ang0 + d_ang * (s / arc_length));
        SolveInverseKinematic(target, m_angles_v[k - 1], m_angles_v[k]);
        Vector3 tangent = w * cos(phi) - u * sin(phi);
        m_speed_v[k] = tangent * profile.Vel(t);
        m_accel_v[k] = tangent * profile.Acc(t);
    }
    DifferentiateJointPath(m_angles_v, last, dt_s);
    m_angles_v[last + 1] = m_angles_v[last];

    CalcCollisionsDynamics(last + 1, dt_millisec);
    AnglesVectorToVector(last + 1, results);
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
            ThrowOnCollision(size, dt_millisec);
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

int Solver::FindMeshBelowFloor() const
{
    //the base stands on the floor; the obstacle is not part of the arm
    for (int i = 0; i < int(m_parts.size()); i++)
    {
        if (m_parts[i].body < 0 || m_bounds[i].point_min.z >= m_floor_z)
            continue;
        for (ushort k = 0; k < m_meshes_buff[i].Size(); k++)
            if (m_meshes_buff[i].point(k).z < m_floor_z)
                return i;
    }
    return -1;
}

const vector<ArmPart> &Solver::GetArmParts() const
{
    return m_parts;
}

vector<Robo::Moments> Solver::GetTrajectoryMoments() const
{
    const size_t n = std::min<size_t>(m_moments_counter, m_moments_v.size());
    vector<Robo::Moments> out(m_moments_v.begin(), m_moments_v.begin() + n);
    //CheckMaxMoments throws before the sample is stored; m_moments still holds it
    if (n > 0)
        out.back() = m_moments;
    return out;
}

const vector<pair<scalar, scalar>> &Solver::GetJointLimits() const
{
    return m_joint_limits;
}

bool Solver::WithinJointLimits(const Angles &a) const
{
    for (int i = 0; i < JOINTS_COUNT; i++)
        if (a.q[i] < m_joint_limits[i].first || a.q[i] > m_joint_limits[i].second)
            return false;
    return true;
}

scalar Solver::GetFloorHeight() const
{
    return m_floor_z;
}

void Solver::SetFloorHeight(scalar z)
{
    m_floor_z = z;
}

Angles Solver::GetExtarnalPosition()
{
    Angles start_angle_external;
    start_angle_external.q[1] = -M_PI/4;
    start_angle_external.q[2] = +M_PI/2;
    start_angle_external.q[3] = -M_PI + M_PI/3.7;
    start_angle_external.q[4] = -M_PI/2;
    return start_angle_external;
}

Angles Solver::GetInternalPosition()
{
    Angles start_angle_internal;
    start_angle_internal.q[1] = -M_PI/4;
    start_angle_internal.q[2] = +M_PI/2;
    start_angle_internal.q[3] = +M_PI/4;
    start_angle_internal.q[4] = +M_PI/2;
    return start_angle_internal;
}

Angles Solver::GetZeroPosition()
{
    return Angles();
}
