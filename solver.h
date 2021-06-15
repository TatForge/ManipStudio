#ifndef SOLVER_H
#define SOLVER_H
#include "robo.h"
#include "emulator.h"
#include "meshcollider.h"

#include <condition_variable>
#include <queue>

using namespace rm;

struct BoxAABB
{
    BoxAABB(const scalar &x_min, const scalar &y_min, const scalar &z_min, const scalar &x_max, const scalar &y_max, const scalar &z_max);
    Vector3 point_max;
    Vector3 point_min;
};

struct IKData
{
    Quaternion end_eff_target_ori;
    Quaternion end_eff_world_ori;
    Quaternion deltaQ;
    Matrix4x4 TR[6];
    Vector3 ang_vel;
    Vector3 delta_r;
    Vector3 delta_s;
    Vector3 axis;
    Vector3 maxis;
    Matrix jacobian = Matrix(6,6);
    Matrix jacobian_transposed = Matrix(6, 6);
    Matrix U = Matrix(6, 6);
    Matrix aug_matrix = Matrix(6, 7);
    Angles d_thet;
    Vector delta_t = Vector(6);
    Vector delta_thetas = Vector(6);
    counter ctr = 0;
    scalar angle = 0;
};


class Solver
{
public:
    Solver(Robo &robot);
    void SolveDirectKinematic(const Angles &q, Matrix4x4 TR[], Matrix4x4 *T = nullptr) noexcept;
    void SolveInverseKinematic(const StateVector &target, const Angles &curr_angle, Angles &res_angle, IKData *data  = nullptr);
    void MoveL(const StateVector &start_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const Angles &start_angles, const ushort &dt_millisec, vector<float> results);
    bool CheckTrajectoryForLinear(const Polynom &pol, const Vector3 &l1, const Vector3 &l2, const scalar &t_millisec, const scalar &max_delta);
    bool CheckTrajectoryForLinear(const Angles angles, const Vector3 &l1, const Vector3 &l2, const scalar &max_delta);
    bool MoveMeshes(const Matrix4x4 TR[]) noexcept;
    void CalcUMatrix() noexcept;
    //Multithread
    void SolveForwardDynamicMultithread();
    void CheckCollisionMultithread();
    bool SolveForwardDynamic(const Angles &angles);
    void VisualizeMoments(const scalar &dt_millisec)
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
    inline void AnglesVectorToVector(const int size, vector<float> &results)
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

    void MoveJ(const Angles &start_angle, const Angles &end_angle, const float &speed, const float &acceleration, const float &dt_millisec, vector<float> &results)
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
    };
    int CalcTrajOneJoint(const Angles &start_angle, const Angles &end_angle, const scalar &dt_s, const scalar &angle_max_speed, const scalar &angle_acceleration, const int &element_number)
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
    void VectorsResize(const counter &ctr);
    void CheckCollision(const uint *elem_number = nullptr)
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
    bool MoveCircle(const Angles &start_angle, const StateVector &start_point, const StateVector &mid_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const scalar &dt_millisec, vector<float> &results)
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
        scalar t, t1, v_s, a_s;
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
            t1 = (AB2.x * N1.y - AB2.y * N1.x - BC2.x * N1.y + BC2.y * N1.x) / (N1.y * N2.x - N1.x * N2.y);
        }
        else if ((N1.z * N2.x - N1.x * N2.z) != 0)
        {
            t  = (AB2.x * N2.z - AB2.z * N2.x - BC2.x * N2.z + BC2.z * N2.x) / (N1.z * N2.x - N1.x * N2.z);
            t1 = (AB2.x * N1.z - AB2.z * N1.x - BC2.x * N1.z + BC2.z * N1.x) / (N1.z * N2.x - N1.x * N2.z);
        }
        else if ((N1.z * N2.y - N1.y * N2.z) != 0)
        {
            t  = (AB2.y * N2.z - AB2.z * N2.y - BC2.y * N2.z + BC2.z * N2.y) / (N1.z * N2.y - N1.y * N2.z);
            t1 = (AB2.y * N1.z - AB2.z * N1.y - BC2.y * N1.z + BC2.z * N1.y) / (N1.z * N2.y - N1.y * N2.z);
        }

        O = Vector3(N1.x * t + AB2.x, N1.y * t + AB2.y, N1.z * t + AB2.z);
        R = A - O;
        auto GetPoint = [&](const scalar &ang){return Vector3(N1/N1.Length() * R.Length() * sin(ang) + N3/N3.Length() * R.Length() * cos(ang) + O);};
        //Угол между начальной точкой и точкой P
        Vector3 P0;
        P0 = GetPoint(0);
        auto GetAngle = [&](const Vector3 &V){
//            auto ch = P0.x * V.x + P0.y * V.y + P0.z * V.z;
//            auto zn = P0.x * V.y * N4.z + V.x * N4.y * P0.z +
//                      N4.x * P0.y * V.z - P0.z * V.y * N4.x -
//                      V.z * N4.y * P0.x - N4.z * P0.y * V.x;
//            auto lol = atan2(zn, ch);
//            auto VO = V - O;
//            auto PO = P0 - O;
            scalar angle = (V - O).Angle(P0 - O);
            if ((GetPoint(angle) - V).Length() > 10e-8)
                angle *= -1;
            return angle;};//(V - O).Angle(P0 - O);};/*atan2(zn, ch);};/*
/*                                                                                  P0.x * V.y * N4.z + V.x * N4.y * P0.x +
                                                                                    N4.x * P0.x * V.z - P0.z * V.y * N4.x -
                                                                                    V.z * N4.y * P0.x - N4.z * P0.y * V.x);};
                                                                                    */
//        scalar Pop = GetAngle(P0);
        current_angle = GetAngle(A);
//        current_angle = GetAngle(P0);
//        auto ii = GetPoint(current_angle);
//        auto i1 = GetPoint(0.1);
//        auto i2 = GetPoint(0.2);
//        auto i3 = GetPoint(0.3);
//        auto i4 = GetPoint(0.4);
//        auto i5 = GetPoint(0.5);
//        auto i6 = GetPoint(0.6);
//        auto i7 = GetPoint(0.7);
//        auto i8 = GetPoint(0.8);
//        auto i9 = GetPoint(0.9);
//        auto i10 = GetPoint(1.0);
//        auto i11 = GetPoint(1.1);
//        auto i12 = GetPoint(1.2);
//        auto i13 = GetPoint(1.3);
//        auto k1 = GetAngle(i13);
//        auto aa = GetPoint(current_angle) - A;
        if ((GetPoint(current_angle) - A).Length() < 10e-6)
            direction = 1;
        else
            direction = -1;
        bool breaking(0);
//        scalar end_angle = GetAngle(end_point.GetCoord());
        angle_range = (start_point.GetAngles() - end_point.GetAngles());
        current_range = (C - A).Length();
        end_angle = GetAngle(end_point.GetCoord());
        start_angle_range = (end_point.GetCoord() - O).Angle(A - O);
        auto llk = v.Length() * dt_s + a_tcp * dt_s * dt_s / 2;
        while(current_range > (v.Length() * dt_s - a_tcp * dt_s * dt_s / 2))
        {
            auto llk = v.Length() * dt_s - a_tcp * dt_s * dt_s / 2;
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
                auto ok = (v.Length() * dt_s + a_tcp * dt_s * dt_s / 2) / R.Length();
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
            auto kok = GetAngle(point);
//            distance_angle_to_stop = abs(end_angle + direction * GetAngle(point));
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
        m_emulator->moments_window.PlotCoordinates(m_moments.tao[0], m_moments.tao[1],m_moments.tao[2],m_moments.tao[3],m_moments.tao[4],m_moments.tao[5], dt_millisec, true);
        m_emulator->MoveManipLinear(m_angles_v, m_speed_v, dt_millisec);
        return true;
    }
    void CalcCollisionsDynamics(const scalar &array_size, const scalar &dt_millisec)
    {
        Matrix4x4 buff[6];
        scalar dt_s = ToSec(dt_millisec);
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
    inline scalar FindAngle(const Vector3 &O, const Vector3 &A0, const Vector3 P)
    {
        Vector3 A0O;
        Vector3 PO;
        A0O = A0 - O;
        PO = P - O;
        return acos(A0O.Dot(PO) / (A0O.Length() * PO.Length()));
    }
private:
    Solver(Solver &solver) = delete;
    Solver(Solver &&solver) = delete;
    bool CheckAABB(const BoxAABB &box1, const BoxAABB &box2);
    void CalculateDeltas();
    const Matrix *manip_config;         //Матрица хранит конфигурацию манипулятора, не текущее состояние
    Matrix4x4 m_T[JOINTS_COUNT];          //buff matrix for calculations
    Emulator *m_emulator;
    std::vector<MeshCollider> m_meshes;
    std::vector<MeshCollider> m_meshes_buff;
    //Dynamic___________________
    Robo::Moments m_moments;
    array<Tensor,6> m_inertion;
    std::vector<scalar> m_mass;
    Matrix4x4 U[36];
    queue<exception_ptr> exception_queue;
    Vector m_gv{0, 0, -g, 0};
    array<rm::Vector, JOINTS_COUNT> m_cm;
    array<rm::Tensor, JOINTS_COUNT> m_i;
    array<rm::Matrix4x4, JOINTS_COUNT> m_ji;

    vector<Angles>  m_angles_v;
    vector<Vector3> m_speed_v;
    vector<Vector3> m_accel_v;
    uint vector_size = 500;
    vector<Robo::Moments> m_moments_v;
    uint m_moments_counter;
    uint m_collisions_counter;
    IKData *m_main_IKData;

    //    Angles m_buff_angles_last;
    //    Angles m_buff_angles_previous;
#if MULTITHREAD
    IKData *m_dynamic_IKData;
    thread *m_dynamic_thread;
    thread *m_collision_thread;
    thread *m_visual_thread;
    condition_variable m_dynamic_cv;
    condition_variable m_collision_cv;
    condition_variable m_visual_cv;
    bool m_dynamic_notified;
    bool m_collision_notified;
    bool m_visual_notified;
    mutex m_dynamic_mtx;
    mutex m_collision_mtx;
    mutex m_visual_mtx;
#endif
    //__________________________
//    GJKCollision *collider;
//    Matrix4x4 T0[JOINTS_COUNT];
};

#endif // SOLVER_H
