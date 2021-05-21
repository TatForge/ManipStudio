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
    Solver(Robo &manip);
    void SolveDirectKinematic(const Angles &q, Matrix4x4 TR[], Matrix4x4 *T = nullptr) noexcept;
    void SolveInverseKinematic(const scalar &accuracy, const StateVector &target, const Angles &curr_angle, Angles &res_angle, IKData *data  = nullptr);
    void CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const Angles &start_angles, Angles &res_angle, const ushort &dt_millisec, Polynom result[]);
    bool CheckTrajectoryForLinear(const Polynom &pol, const Vector3 &l1, const Vector3 &l2, const scalar &t_millisec, const scalar &max_delta);
    bool CheckTrajectoryForLinear(const Angles angles, const Vector3 &l1, const Vector3 &l2, const scalar &max_delta);
    bool MoveMeshes(const Matrix4x4 TR[]) noexcept;
    void CalcUMatrix() noexcept;
    //Multithread
    void SolveForwardDynamicMultithread();
    void CheckCollisionMultithread();
    bool SolveForwardDynamic(const Angles &angles);

    void MoveJ(const Angles &start_angle, const Angles &end_angle, const float &speed, const float &acceleration, const float &dt_millisec)
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
        for (int i = 0; i < 6; i++)
        {
            size_of_arr = CalcTrajOneJoint(start_angle, end_angle, dt_s, speed, acceleration, i);
            sizes[i] = size_of_arr;
            if (max_size < size_of_arr)
                max_size = size_of_arr;
            if (min_size < size_of_arr)
                min_size = size_of_arr;
        }
        m_angles_v.resize(max_size);
        m_speed_v.resize(max_size);
        m_accel_v.resize(max_size);
        for (counter i = 0; i < 6; i++)
        {
            for (counter j = sizes[i] + 1; j < max_size; j++)
                m_angles_v[j].q[i] = m_angles_v[sizes[i]].q[i];
//            std::fill(m_angles_v.begin()->q[i] + sizes[i], m_angles_v.end()->q[i], m_angles_v[sizes[i]].q[i]);
        }

        //Коллизии + динамика
        CalcCollisionsDynamics(max_size, dt_millisec);
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
//            #if WITH_WINDOW
//            m_emulator->moments_window.PlotCoordinates(m_moments.tao[0], m_moments.tao[1],m_moments.tao[2],m_moments.tao[3],m_moments.tao[4],m_moments.tao[5], dt_millisec);
//            #endif
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
#if WITH_WINDOW
        m_emulator->moments_window.PlotCoordinates(m_moments.tao[0], m_moments.tao[1],m_moments.tao[2],m_moments.tao[3],m_moments.tao[4],m_moments.tao[5], dt_millisec, true);
        m_emulator->MoveManipLinear(m_angles_v, m_speed_v, dt_millisec);
#endif
    };
    int CalcTrajOneJoint(const Angles &start_angle, const Angles &end_angle, const scalar &dt_s, const scalar &angle_max_speed, const scalar &angle_acceleration, const int &element_number)
    {
        //Calculate trajectory for one joint and write result into array
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
    bool MoveCircle(const Angles &start_angle, const StateVector &start_point, const StateVector &mid_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const scalar &dt_millisec)
    {
        scalar dt_s = ToSec(dt_millisec);
        Vector3 A = start_point.GetCoord();
        Vector3 B = mid_point.GetCoord();
        Vector3 C = end_point.GetCoord();
        Vector3 AB = A - B;
        Vector3 BC = B - C;
        Vector3 AC = A - C;
        m_moments_vector.resize(vector_size);
        m_angles_v.resize(vector_size);
        m_speed_v.resize(vector_size);
        m_accel_v.resize(vector_size);
        std::vector<Vector3> points;
        points.resize(vector_size);
        m_angles_v[0] = start_angle;
        points[0] = A;
        //Проверка точек на нахождение на одной прямой
        if ((AB.x/AC.x + AB.y/AC.y + AB.z/AC.z) - 3 * (AB.x/AC.x) < 10e-6)
        {
            cout << "Точки лежат на одной прямой" << endl;
            return false;
        }
        //____________________________________________
        Vector3 N = AB * BC;
        Vector3 AB2 = (A + B) / 2;
        Vector3 BC2 = (B + C) / 2;
        Vector3 ABn, BCn, O, R, N3,
                Okr, v, a, n, r;
        //Данная конструкция позволяет избежать вырождения решения в случае, когда нормаль
        //окружности не пересекает плоскость, образованную приравниванием одной из осей единице, например (1, y, z).
//        N = N.Normalize();
//        if (abs(N.y) > 0.1)
//        {
//            ABn.x = 1;
//            ABn.z = (AB.y * N.x - AB.x * N.y) / (AB.z * N.y - AB.y * N.z);
//            ABn.y = -(N.z * ABn.z + N.x) / N.y;
//        } else if (abs(N.x) > 0.1)
//        {
//            ABn.y = 1;
//            ABn.z = (AB.x * N.y - AB.y * N.x) / (AB.z * N.x - AB.x * N.z);
//            ABn.x = -(N.z * ABn.z + N.y) / N.x;
//        }
//        else if (abs(N.z) > 0.1)
//        {
//            ABn.z = 1;
//            ABn.y = (AB.x * N.z - AB.z * N.x) / (AB.y * N.x - AB.x * N.y);
//            ABn.x = -(N.y * ABn.y + N.z) / N.x;
//        } else
//            cout << "WTF if going on? N(0, 0, 0)" << endl;
        Vector3 N1 = N * AB;
        Vector3 N2 = N1 * N;

//        if (N.x == 0 && N.y == 0)
//            ABn.z = 0;
//        else if (N.x == 0 && N.z == 0)
//            ABn.y = 0;
//        else if (N.y == 0 && N.z == 0)
//            ABn.x = 0;

//        ABn.x = 1;
//        ((AB.y * N.x - AB.x * N.y) == 0) ? ABn.z = 0 : ABn.z = (AB.y * N.x - AB.x * N.y) / (AB.z * N.y - AB.y * N.z);
//        (-(N.z * ABn.z + N.x) == 0) ? ABn.y = 0 : ABn.y = -(N.z * ABn.z + N.x) / N.y;
//        BCn.x = 1;
//        ((BC.y * N.x - BC.x * N.y) == 0) ? BCn.z = 0 : BCn.z = (BC.y * N.x - BC.x * N.y) / (BC.z * N.y - BC.y * N.z);
//        ((N.z * BCn.z + N.x) == 0) ? BCn.y = 0 : BCn.y = -(N.z * BCn.z + N.x) / N.y;
//        scalar t = (AB2.x * ABn.y + BC2.y - ABn.y * BC2.x - AB2.y) / (ABn.y - BCn.y);
//        if ()
        scalar t1 = (N.x * (BC2.y - AB2.y) - N.y *(BC2.x + AB2.x)) / (N.y * N1.x - N1.y * N.x);
        scalar t  = (N1.x * t1 + BC2.x + AB2.x) / N.x;
        O = Vector3(N.x * t1 + AB2.x, N.y * t1 + AB2.y, N.z * t1 + AB2.z);

//        scalar t1 = BCn.x * t + BC2.x - AB2.x;
        scalar l;
        scalar alpha;
//        O = Vector3(t1 + AB2.x, ABn.y * t1 + AB2.y, ABn.z * t1 + AB2.z);
        R = A - O;
//        N3;
        N3.z = (N.y - N.x * ABn.y) / (N.z * ABn.y - ABn.z * N.y);
        N3.y = (-1 - ABn.z * N3.z) / ABn.y;
        N3.x = 1;
//        scalar angle(M_PI/2);
//        Okr = ABn/ABn.Length() * R * sin(angle) + N3/N3.Length() * R * sin(angle) + O;
        auto GetPoint = [&](const scalar &ang){return Vector3(ABn/ABn.Length() * R * sin(ang) + N3/N3.Length() * R * sin(ang) + O);};
        Vector3 P0;
        P0 = ABn/ABn.Length() * R * sin(0) + N3/N3.Length() * R * sin(0) + O;
        alpha = (P0 - O).Angle(A - O);
        counter ctr;
        while(1)
        {
            Vector3 point;
            r = A;
            if (v.Length() >= v_tcp)
            {
                alpha += (v_tcp * dt_s) / R.Length();
                point = GetPoint(alpha);

                n = point - points[ctr];
                a = a.Normalize();
                v = a * v_tcp;
                a = 0;
            } else
            {
                alpha = +(v.Length() * dt_s + a_tcp * a_tcp * dt_s / 2) / R.Length();
                point = GetPoint(alpha);

                a = point - points[ctr];
                a = a.Normalize();
                a = a * a_tcp;
                v += a * dt_s;
            }

            l = v.Length() * dt_s + a.Length() * a.Length() * dt_s / 2;
            alpha = l / (2 * M_PI * R.Length());
//            Point = ;
            ctr++;
            points[ctr] = point;
            Vector3 angles;
            SolveInverseKinematic(ACCURACY, StateVector(points[ctr], angles), m_angles_v[ctr - 1], m_angles_v[ctr]);
            m_speed_v[ctr + 1].x = v.x;
            m_speed_v[ctr + 1].y = v.y;
            m_speed_v[ctr + 1].z = v.z;
            m_accel_v[ctr].x = a.x;
            m_accel_v[ctr].y = a.y;
            m_accel_v[ctr].z = a.z;
        }

        CalcCollisionsDynamics(ctr, dt_millisec);
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
            for (uint j = 0; j < JOINTS_COUNT; j++)
            {
                m_angles_v[size].q_pp[j] = (m_angles_v[size + 1].q[j] - m_angles_v[size].q[j] - m_angles_v[size].q_p[j] * dt_s) / (dt_s * dt_s) /*0.1*/;
                m_angles_v[size + 1].q_p[j] = m_angles_v[size].q_p[j] + m_angles_v[size].q_pp[j] * dt_s;
            }
#if !MULTITHREAD
            SolveForwardDynamic(m_angles_v[size]);
            m_moments_vector[size] = m_moments;
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
    vector<Robo::Moments> m_moments_vector;
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
