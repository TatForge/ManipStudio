#include "emulator.h"

//static Vector3 current_range(0, 0, 0);

Emulator::Emulator()
{

}

Emulator::Emulator(const Robo &manip)
{
//#if !BENCHMARK
    w.show();
//#endif
    state_matrix = new Matrix(manip.state_matrix->GetColsCount(), manip.state_matrix->GetRowsCount());
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        T[i] = manip.T[i];
    }
    *state_matrix = *manip.state_matrix;
    m_emu_angles = manip.m_angles;
    shared_manip = new SharedMem("manipulator", 18);
    shared_points = new SharedMem("points", sizeof(points_to_visual) / sizeof(float));
    for (size_t i = 0; i < shared_points->GetSize(); i++)
        points_to_visual[i] = 0;
}

Emulator::~Emulator()
{
    delete state_matrix;
}

void Emulator::MoveManipPolynom(Polynom polynom_arr[], scalar moving_time, scalar dt)
{
//    t = 0;
//    bool is_last = false;
//    while (t <= moving_time)
//    {
//        if (moving_time - t < 10e-4)
//            is_last = true;
//        OneIterMove(is_last);
    //    }
}

void Emulator::MoveManipLinear(Angles angles[], size_t size, scalar dt_msec, Vector3 accel[])
{
    t_m = 0;
    bool is_last = false;
    for (size_t i = 0; i < size; i++)
    {
        if (angles[i+2].IsNull())
            is_last = true;
        OneIterMoveLinear(angles[i], angles[i + 1], dt_msec, is_last, accel[i]);
        if (is_last)
            break;
    }
    w.PlotSpeeds(m_emu_angles);
}

void Emulator::MoveManipLinear(std::vector<Angles> angles, std::vector<Vector3> accel, scalar dt_msec)
{
    t_m = 0;
    bool is_last = false;
    for (size_t i = 0; i < angles.size() - 1; i++)
    {
        if (i == angles.size() - 2)
            is_last = true;
        OneIterMoveLinear(angles[i], angles[i + 1], dt_msec, is_last, accel[i]);
    }
    w.PlotSpeeds(m_emu_angles);

}

void Emulator::MoveManipPolynom(Polynom polynom_in, int moving_time_millisec)
{
//    scalar moving_time = moving_time_millisec / 1000.0;
    SetPolynomKF(polynom_in);
    t_m = 0;
    bool is_last = false;
    while (t_m < moving_time_millisec)
    {
        if (t_m == moving_time_millisec - m_delta_t_millisec)
            is_last = true;
        OneIterMove(is_last);
    }
    w.PlotSpeeds(m_emu_angles,11);
    t_m = 0;
}

void Emulator::SolveDirectKinematic()
{
    auto &A = *state_matrix;
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
    L = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];

    //        std::cout << "Current TCP position is " << "x = " << L.a14 << " y = " << L.a24 << " z = " << L.a34 << " time " << t << std::endl;

//    w.PlotData(roll, pitch, yaw, t);
}

void Emulator::OneIterMove(bool is_last = false)
{
    t_m += m_delta_t_millisec;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        m_emu_angles.q[i]    = m_polynom.GetAngle(i, t_m);
        m_emu_angles.q_p[i]  = m_polynom.GetSpeed(i, t_m);
        m_emu_angles.q_pp[i] = m_polynom.GetAcceleration(i, t_m);
    }
    UpdateStateMatrix();
    SolveDirectKinematic();
    FillManipPositionData();
//    std::this_thread::sleep_for(std::chrono::milliseconds(10));///m_delta_t * 1000));
    std::thread tre1(&Emulator::SendDataToShared, this, MANIP, manip_to_visual, sizeof(float) * 18);
    std::thread tre2(&Emulator::SendDataToShared, this, POINTS, points_to_visual, sizeof(float) * shared_points->GetSize() - 4);
    tre1.detach();
    tre2.detach();
//    SendDataToShared(MANIP, manip_to_visual, sizeof(float) * 18);
//    SendDataToShared(POINTS, points_to_visual, sizeof(float) * shared_points->GetSize() - 4);
//    if (is_last)
//        for (size_t i = 0; i < shared_points->GetSize(); i+=3)
//            if (points_to_visual[i] > 2 || points_to_visual[i + 1] > 2 || points_to_visual[i + 2] > 2)
//                std::cout << "points " << " x = " << points_to_visual[i] << " y = " << points_to_visual[i + 1] << " z = " << points_to_visual[i + 2] << endl;
    auto angles = L.ConvertToQuaternion().GetEulerAngles();
    w.PlotCoordinates(L(0,3), L(1,3), L(2,3), ToDeg(angles.z), ToDeg(angles.y), ToDeg(angles.x), m_delta_t_millisec, is_last);
    w.PlotSpeeds(m_emu_angles, is_last);
    if (is_last)
    {
        w.PrintResult(L(0,3), L(1,3), L(2,3), angles.z, angles.y, angles.x);
    }
}

void Emulator::OneIterMoveLinear(const Angles &angle1, const Angles &angle2, const int &dt_msec, bool is_last, Vector3 accel)
{
    for( int i = 0; i < dt_msec/m_delta_t_millisec; i++)
    {
        this_thread::sleep_for(std::chrono::microseconds(800));
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            m_emu_angles.q[j]    = interpolate(angle1.q[j],    angle2.q[j], (scalar(i) / scalar(dt_msec/m_delta_t_millisec)));
            m_emu_angles.q_p[j]  = interpolate(angle1.q_p[j],  angle2.q_p[j], (scalar(i) / scalar(dt_msec/m_delta_t_millisec)));
            m_emu_angles.q_pp[j] = interpolate(angle1.q_pp[j], angle2.q_pp[j], (scalar(i) / scalar(dt_msec/m_delta_t_millisec)));
        }
        UpdateStateMatrix();
        SolveDirectKinematic();
        FillManipPositionData();
        SendDataToShared(MANIP, manip_to_visual, sizeof(float) * 18);
        SendDataToShared(POINTS, points_to_visual, sizeof(float) * shared_points->GetSize() - 4);
        auto angles = L.ConvertToQuaternion().GetEulerAngles();
//        auto ii = Vector3(accel[(t_m)/dt_msec].x, accel[(t_m)/dt_msec].y, accel[(t_m)/dt_msec].z);
//        w.PlotCoordinates(L(0,3), L(1,3), L(2,3), /*ToDeg(angles.z), ToDeg(angles.y), ToDeg(angles.x),*/
//                          accel[(t_m)/dt_msec].x, accel[(t_m)/dt_msec].y, accel[(t_m)/dt_msec].z, m_delta_t_millisec, is_last);
        w.PlotCoordinates(L(0,3), L(1,3), L(2,3), accel.x, accel.y, accel.z, m_delta_t_millisec, is_last);

        w.PlotSpeeds(m_emu_angles, is_last);
        if (is_last)
        {
            w.PrintResult(L(0,3), L(1,3), L(2,3), angles.z, angles.y, angles.x);
        }
        t_m += m_delta_t_millisec;
    }

//    if (is_last)
//        for (size_t i = 0; i < shared_points->GetSize(); i+=3)
//            if (points_to_visual[i] > 2 || points_to_visual[i + 1] > 2 || points_to_visual[i + 2] > 2)
//                std::cout << "points " << " x = " << points_to_visual[i] << " y = " << points_to_visual[i + 1] << " z = " << points_to_visual[i + 2] << endl;
//    auto angles = L.ConvertToQuaternion().GetEulerAngles();
//    w.PlotCoordinates(L(0,3), L(1,3), L(2,3), ToDeg(angles.z), ToDeg(angles.y), ToDeg(angles.x), m_delta_t_millisec, is_last);
//    w.PlotSpeeds(m_emu_angles, m_delta_t_millisec, is_last);
//    if (is_last)
//    {
//        w.PrintResult(L(0,3), L(1,3), L(2,3), angles.z, angles.y, angles.x);
//    }
}

void Emulator::SendDataToShared(SharedMemKey skey, float *data_ptr, size_t size)
{
    data_to_shared_mutex.lock();
    if (skey == 1)
    {
        if (!shared_manip->SendData(data_ptr, size))
            throw (QString("can't send data to shared memory(shared_manip)"));
    }
    if (skey == 2)
    {
        if (!shared_points->SendData(data_ptr, size))
            throw (QString("can't send data to shared memory(shared_points)"));
    }
    data_to_shared_mutex.unlock();
}

void Emulator::FillManipPositionData()
{
    RoboMath::Matrix4x4 T_b;
    T_b.MakeIdentity();
    for (int i = 0; i < 6; i++)
    {
        T_b = T_b * T[i];
        manip_to_visual[i * 3 + 0] = T_b(0, 3);
        manip_to_visual[i * 3 + 1] = T_b(1, 3);
        manip_to_visual[i * 3 + 2] = T_b(2, 3);
    }
    if (iter == 0)
    {
        points_to_visual[iter + 0] = T_b(0, 3);
        points_to_visual[iter + 1] = T_b(1, 3);
        points_to_visual[iter + 2] = T_b(2, 3);
        iter += 3;
    }
    else if (sqrt(pow(points_to_visual[iter - 3] - T_b(0,3) , 2) + pow(points_to_visual[iter - 2]
                - T_b(1,3), 2) + pow(points_to_visual[iter - 1] - T_b(2,3), 2)) > RANGE_TO_PLOT)
    {
        points_to_visual[iter + 0] = T_b(0, 3);
        points_to_visual[iter + 1] = T_b(1, 3);
        points_to_visual[iter + 2] = T_b(2, 3);
        iter += 3;
    }
}

void Emulator::SetPolynomKF(Polynom polynom_in)
{
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        m_polynom = polynom_in;
    }
}

void Emulator::UpdateStateMatrix()
{
    for (int i = 0; i < JOINTS_COUNT; i++)
        (*state_matrix)(i, 3) = m_emu_angles.q[i];
}



//void Emulator::OneTick()
//{
//    for (int i = 0; i < JOINTS_COUNT; i++)
//    {
//        //t - текущее время
//        t += m_delta_t;
//        scalar v = (m_polynom + i)->elem[5] * pow(t, 5) + (m_polynom + i)->elem[4] * pow(t, 4) + (m_polynom + i)->elem[3] * pow(t, 3) +
//                (m_polynom + i)->elem[2] * pow(t, 2) + (m_polynom + i)->elem[1] * pow(t, 1) + (m_polynom + i)->elem[0];
//        m_emu_angles.q[i] += v * m_delta_t;
//    }
//    UpdateStateMatrix();
//}
