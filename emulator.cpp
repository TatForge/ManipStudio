#include "emulator.h"

Emulator::Emulator()
{

}

Emulator::Emulator(const Robo &manip)
{
    //        moveToThread(&m_thread);

    //        connect(&m_timer, &QTimer::timeout, this, &Emulator::OneTick);
    //        connect(&m_thread, &QThread::started, this, &Emulator::startTimer);
    //        this->moveToThread(&m_thread);
    w.show();
    state_matrix = new Matrix(manip.state_matrix->GetColsCount(), manip.state_matrix->GetRowsCount());
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        T[i] = manip.T[i];
    }
    *state_matrix = *manip.state_matrix;
    m_emu_angles = manip.m_angles;
    m_polynom = new Polynom [JOINTS_COUNT];
}

Emulator::~Emulator()
{
    delete[] state_matrix;
    delete[] m_polynom;
}

void Emulator::MoveManip(Polynom *polynom_in, scalar moving_time)
{
    SetPolynomKF(polynom_in);
    t = 0;
    while (t <= moving_time)
    {
        OneIterMove();
    }
    double pitch = atan(L.a31/L.a32);
    double yaw   = atan(L.a13/L.a23);
    double roll  = atan((sin(pitch) * L.a31 - cos(pitch) * L.a32)/L.a33);
    w.PrintResult(L.a14, L.a24, L.a34, roll, pitch, yaw, t);
    t = 0;
    //        m_thread.start();
    //        m_timer.start();
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
    double pitch = atan(L.a31/L.a32);
    double yaw   = atan(L.a13/L.a23);
    double roll  = atan((sin(pitch) * L.a31 - cos(pitch) * L.a32)/L.a33);
    w.PlotData(L.a14, L.a24, L.a34, t);
//    w.PlotData(roll, pitch, yaw, t);
}

void Emulator::SolveInverseKinematic()
{

}

void Emulator::OneIterMove()
{
    scalar v;
    //        this_thread::sleep_for(std::chrono::milliseconds(int(m_delta_t * 1000)));
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
//        m_emu_angles.q[i] = (m_polynom + i)->elem[5] * pow(t, 5) + (m_polynom + i)->elem[4] * pow(t, 4) + (m_polynom + i)->elem[3] * pow(t, 3) +
//                            (m_polynom + i)->elem[2] * pow(t, 2) + (m_polynom + i)->elem[1] * pow(t, 1) + (m_polynom + i)->elem[0];
//        v = (m_polynom + i)
        v = 5 * (m_polynom + i)->elem[5] * pow(t, 4) + 4 * (m_polynom + i)->elem[4] * pow(t, 3) + 3 * (m_polynom + i)->elem[3] * pow(t, 2) +
            2 * (m_polynom + i)->elem[2] * pow(t, 1) + (m_polynom + i)->elem[1];
        m_emu_angles.q[i] += v * m_delta_t;
    }
    UpdateStateMatrix();
    SolveDirectKinematic();
    t += m_delta_t;
}

void Emulator::SetPolynomKF(Polynom *polynom_in)
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
