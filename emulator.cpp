#include "emulator.h"

//static Vector3 current_range(0, 0, 0);

Emulator::Emulator()
{

}

Emulator::Emulator(const Robo &manip)
{
//#if !BENCHMARK
#if WITH_WINDOW
    w.show();
    moments_window.show();
#endif
    state_matrix = new Matrix(manip.state_matrix->GetColsCount(), manip.state_matrix->GetRowsCount());
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        T[i] = manip.T[i];
    }
    *state_matrix = *manip.state_matrix;
    m_emu_angles = manip.m_angles;
//    shared_manip = new SharedMemory(FILE_MANIP_SHM, 18);
//    shared_points = new SharedMemory(FILE_POINTS_SHM, sizeof(points_to_visual) / sizeof(float));
//    shared_meshes = new SharedMemory(FILE_MODELS_SHM, sizeof(meshes_to_visual) / sizeof(float));
    shared_manip = new SharedMemory("manipulator", 18);
    shared_points = new SharedMemory("points", sizeof(points_to_visual) / sizeof(float));
    shared_meshes = new SharedMemory("meshes", sizeof(meshes_to_visual) / sizeof(float));
    for (size_t i = 0; i < shared_points->GetSize(); i++)
        points_to_visual[i] = 0;
#ifdef VREMENNO_DLYA_VISUALKI
    meshes.resize(9);
    try {
        meshes[0].ReadStl("STLmodels/base.stl");
        meshes[1].ReadStl("STLmodels/joint2.stl");
        meshes[2].ReadStl("STLmodels/link2.stl");
        meshes[3].ReadStl("STLmodels/joint3.stl");
        meshes[4].ReadStl("STLmodels/link3.stl");
        meshes[5].ReadStl("STLmodels/joint4.stl");
        meshes[6].ReadStl("STLmodels/joint5.stl");
        meshes[7].ReadStl("STLmodels/joint6.stl");
        meshes[8].ReadStl("STLmodels/cube0.5m.stl");
        meshes[8].MoveMesh(Quaternion(0, 0, 0, 1), Vector3(SMESH, SMESH, 0));
        meshes_buff = meshes;
    }  catch (std::string &exc) {
        cout << exc << endl;
    }
#endif
}

Emulator::~Emulator()
{
    delete state_matrix;
}
void Emulator::MoveManipLinear(const Angles angles[], size_t size, scalar dt_msec, const Vector3 accel[])
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
#if WITH_WINDOW
    w.PlotSpeeds(m_emu_angles);
#endif
}

void Emulator::MoveManipLinear(std::vector<Angles> angles, std::vector<Vector3> speed, scalar dt_msec)
{
    t_m = 0;
    bool is_last = false;
//    m_emu_angles = angles[0];
    for (size_t i = 0; i < angles.size() - 1; i++)
    {
        if (i == angles.size() - 2)
            is_last = true;
        OneIterMoveLinear(angles[i], angles[i + 1], dt_msec, is_last, speed[i]);
    }
#if WITH_WINDOW
    w.PlotSpeeds(m_emu_angles);
#endif
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

#if WITH_WINDOW
    w.PlotSpeeds(m_emu_angles,11);
#endif
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
//    std::thread tre1(&Emulator::SendDataToShared, this, MANIP, manip_to_visual, sizeof(float) * 18);
//    std::thread tre2(&Emulator::SendDataToShared, this, POINTS, points_to_visual, sizeof(float) * shared_points->GetSize() - 4);
//    tre1.detach();
//    tre2.detach();
//    SendDataToShared(MANIP, manip_to_visual, sizeof(float) * 18);
//    SendDataToShared(POINTS, points_to_visual, sizeof(float) * shared_points->GetSize() - 4);
//    if (is_last)
//        for (size_t i = 0; i < shared_points->GetSize(); i+=3)
//            if (points_to_visual[i] > 2 || points_to_visual[i + 1] > 2 || points_to_visual[i + 2] > 2)
//                std::cout << "points " << " x = " << points_to_visual[i] << " y = " << points_to_visual[i + 1] << " z = " << points_to_visual[i + 2] << endl;
    auto angles = L.ConvertToQuaternion().GetEulerAngles();

#if WITH_WINDOW
    w.PlotCoordinates(L(0,3), L(1,3), L(2,3), ToDeg(angles.z), ToDeg(angles.y), ToDeg(angles.x), m_delta_t_millisec, is_last);
    w.PlotSpeeds(m_emu_angles, is_last);
    if (is_last)
    {
        w.PrintResult(L(0,3), L(1,3), L(2,3), angles.z, angles.y, angles.x);
    }
#endif
}

void Emulator::OneIterMoveLinear(const Angles &angle1, const Angles &angle2, const int &dt_msec, bool is_last, Vector3 accel)
{
    for(int i = 0; i < dt_msec/m_delta_t_millisec; i++)
    {
        this_thread::sleep_for(std::chrono::microseconds(1000));
        for (int j = 0; j < JOINTS_COUNT; j++)
        {
            m_emu_angles.q[j]    = interpolate(angle1.q[j],    angle2.q[j], (scalar(i) / scalar(dt_msec/m_delta_t_millisec)));
            m_emu_angles.q_p[j]  = interpolate(angle1.q_p[j],  angle2.q_p[j], (scalar(i) / scalar(dt_msec/m_delta_t_millisec)));
            m_emu_angles.q_pp[j] = interpolate(angle1.q_pp[j], angle2.q_pp[j], (scalar(i) / scalar(dt_msec/m_delta_t_millisec)));
        }
        UpdateStateMatrix();
        SolveDirectKinematic();
        FillManipPositionData();
#ifdef VREMENNO_DLYA_VISUALKI
        MoveThatShit();
#endif
//        SendDataToShared(MANIP, manip_to_visual, 18);
//        SendDataToShared(POINTS, points_to_visual, shared_points->GetSize() - 4);
//        SendDataToShared(MESHES, meshes_to_visual, shared_meshes->GetSize() - 4);
        SendDataToShared(MANIP, manip_to_visual, sizeof(float) * 18);
        SendDataToShared(POINTS, points_to_visual, sizeof(float) * shared_points->GetSize() - 4);
        SendDataToShared(MESHES, meshes_to_visual, sizeof(float) * shared_meshes->GetSize() - 4);
        auto angles = L.ConvertToQuaternion().GetEulerAngles();
//        auto ii = Vector3(accel[(t_m)/dt_msec].x, accel[(t_m)/dt_msec].y, accel[(t_m)/dt_msec].z);
//        w.PlotCoordinates(L(0,3), L(1,3), L(2,3), /*ToDeg(angles.z), ToDeg(angles.y), ToDeg(angles.x),*/
//                          accel[(t_m)/dt_msec].x, accel[(t_m)/dt_msec].y, accel[(t_m)/dt_msec].z, m_delta_t_millisec, is_last);

#if WITH_WINDOW
        w.PlotCoordinates(L(0,3), L(1,3), L(2,3), accel.x, accel.y, accel.z, m_delta_t_millisec, is_last);

        w.PlotSpeeds(m_emu_angles, is_last);
        if (is_last)
        {
//            w.PrintResult(L(0,3), L(1,3), L(2,3), angles.z, angles.y, angles.x);
        }
#endif
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
    switch (skey)
    {
    case 1:
        if (!shared_manip->SendData(data_ptr, size))
            throw (QString("can't send data to shared memory(shared_manip)"));
        break;
    case 2:
        if (!shared_points->SendData(data_ptr, size))
            throw (QString("can't send data to shared memory(shared_points)"));
        break;
    case 3:
        if (!shared_meshes->SendData(data_ptr, size))
            throw (QString("can't send data to shared memory(shared_meshes)"));
        break;
    }/*
    if (skey == 1)
    {
        if (!shared_manip->SendData(data_ptr, size))
            throw (QString("can't send data to shared memory(shared_manip)"));
    }
    if (skey == 2)
    {
        if (!shared_points->SendData(data_ptr, size))
            throw (QString("can't send data to shared memory(shared_points)"));
    }*/
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

void Emulator::MoveThatShit()
{
    for (uint i = 0; i < 999; i++)
    {
        meshes_to_visual[i] = 0;
    }
    Matrix4x4 TR[6];
    TR[0] = T[0];
    TR[1] = T[0] * T[1];
    TR[2] = T[0] * T[1] * T[2];
    TR[3] = T[0] * T[1] * T[2] * T[3];
    TR[4] = T[0] * T[1] * T[2] * T[3] * T[4];
    TR[5] = T[0] * T[1] * T[2] * T[3] * T[4] * T[5];
    std::vector<Vector3>v_move;
    std::vector<Quaternion> quat_move;
    v_move.push_back(Vector3(0, 0, 0));
    v_move.push_back(TR[0].ConvertToVector3());
    v_move.push_back(TR[1].ConvertToVector3());
    v_move.push_back(TR[2].ConvertToVector3());
    v_move.push_back(TR[3].ConvertToVector3());
    v_move.push_back(TR[4].ConvertToVector3());
    v_move.push_back(TR[5].ConvertToVector3());

    quat_move.push_back(TR[0].ConvertToQuaternion());
    quat_move.push_back(TR[1].ConvertToQuaternion());
    quat_move.push_back(TR[2].ConvertToQuaternion());
    quat_move.push_back(TR[3].ConvertToQuaternion());
    quat_move.push_back(TR[4].ConvertToQuaternion());
    quat_move.push_back(TR[5].ConvertToQuaternion());

    //пересчет точек тут же
    meshes_buff = meshes;

    meshes_buff[0].MoveMesh(quat_move[0], Vector3(0, 0, 0));
    meshes_buff[1].MoveMesh(quat_move[0], v_move[1]);
    meshes_buff[2].MoveMesh(quat_move[1], v_move[2]);
    meshes_buff[3].MoveMesh(quat_move[2], v_move[3]);
    meshes_buff[4].MoveMesh(quat_move[2], v_move[3]);
    meshes_buff[5].MoveMesh(quat_move[3], v_move[4]);
    meshes_buff[6].MoveMesh(quat_move[4], v_move[4]);
    meshes_buff[7].MoveMesh(quat_move[5], v_move[5]);
    uint counter(0);

    for (uint i = 0; i < meshes_buff.size(); i++)
    {
        for (uint j = 0; j < meshes_buff[i].Size(); j++)
        {
            meshes_to_visual[counter + 0] = meshes_buff[i].point(j).x;
            meshes_to_visual[counter + 1] = meshes_buff[i].point(j).y;
            meshes_to_visual[counter + 2] = meshes_buff[i].point(j).z;
            counter+= 3;
        }
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
