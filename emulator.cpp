#include "emulator.h"

//static Vector3 current_range(0, 0, 0);

Emulator::Emulator()
{

}

Emulator::Emulator(const Robo &manip)
{
    w.show();
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

void Emulator::MoveManip(Polynom polynom_arr[], scalar moving_time, scalar dt)
{
    t = 0;
    bool is_last = false;
    while (t <= moving_time)
    {
        if (moving_time - t < 10e-4)
            is_last = true;
        OneIterMove(is_last);
    }
}

void Emulator::MoveManip(Polynom polynom_in, int moving_time_millisec)
{
    scalar moving_time = moving_time_millisec / 1000.0;
    SetPolynomKF(polynom_in);
    t = 0;
    bool is_last = false;
    while (t <= moving_time)
    {
        if (moving_time - t < 10e-4)
            is_last = true;
        OneIterMove(is_last);
    }
    w.PlotSpeeds(m_emu_angles,11);
    t = 0;
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
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        m_emu_angles.q[i]    = m_polynom.GetAngle(i, t);
        m_emu_angles.q_p[i]  = m_polynom.GetSpeed(i, t);
        m_emu_angles.q_pp[i] = m_polynom.GetAcceleration(i, t);
    }
    UpdateStateMatrix();
    SolveDirectKinematic();
    FillManipPositionData();
//    std::this_thread::sleep_for(std::chrono::milliseconds(10));///m_delta_t * 1000));
    SendDataToShared(MANIP, manip_to_visual, sizeof(float) * 18);
    SendDataToShared(POINTS, points_to_visual, sizeof(float) * shared_points->GetSize() - 4);
    if (is_last)
        for (size_t i = 0; i < shared_points->GetSize(); i+=3)
            if (points_to_visual[i] > 2 || points_to_visual[i + 1] > 2 || points_to_visual[i + 2] > 2)
                std::cout << "points " << " x = " << points_to_visual[i] << " y = " << points_to_visual[i + 1] << " z = " << points_to_visual[i + 2] << endl;
    auto angles = L.ConvertToQuaternion().GetEulerAngles();
    w.PrintResult(L(0,3), L(1,3), L(2,3), angles.z, angles.y, angles.x, t);
    w.PlotCoordinates(L(0,3), L(1,3), L(2,3), ToDeg(angles.z), ToDeg(angles.y), ToDeg(angles.x), t, is_last);
    if (is_last)
        w.PlotSpeeds(m_emu_angles, t, is_last);
    t += m_delta_t;
}

void Emulator::SendDataToShared(SharedMemKey skey, float *data_ptr, size_t size)
{
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
    auto ll = RANGE_TO_PLOT;
    auto ii = T_b(0,3);
    auto i1 = T_b(1,3);
    auto i12 = T_b(2,3);
    auto kk = pow(T_b(0,3), 2);
    kk = pow(T_b(1,3), 2);
    kk = pow(T_b(2,3), 2);
    kk = sqrt(pow(T_b(0,3), 2) + pow(T_b(1,3), 2) + pow(T_b(2,3), 2));
//    ll = sqrt(kk);
    ll = sqrt(pow(points_to_visual[iter - 3] - T_b(0,3) , 2) + pow(points_to_visual[iter - 2] - T_b(1,3), 2) + pow(points_to_visual[iter - 1] - T_b(2,3), 2));
//    ll = sqrt(pow(T_b(0,3), 2) + pow(T_b(1,3), 2) + pow(T_b(2,3), 2));
    if (iter == 0)
    {
        points_to_visual[iter + 0] = T_b(0, 3);
        points_to_visual[iter + 1] = T_b(1, 3);
        points_to_visual[iter + 2] = T_b(2, 3);
        iter += 3;
    }
//    else if (abs(sqrt(pow(points_to_visual[iter - 3], 2) + pow(points_to_visual[iter - 2], 2) + pow(points_to_visual[iter - 1], 2)) -
//                 sqrt(pow(T_b(0,3), 2) + pow(T_b(1,3), 2) + pow(T_b(2,3), 2))) > RANGE_TO_PLOT)

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
