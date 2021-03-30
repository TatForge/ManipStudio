#ifndef SOLVER_H
#define SOLVER_H
#include "robo.h"
#include "emulator.h"
#include "meshcollider.h"

//#include "gjkcollision3d.h"
//#include "my_math.h"



using namespace rm;

struct BoxAABB
{
    BoxAABB(const scalar &x_min, const scalar &y_min, const scalar &z_min, const scalar &x_max, const scalar &y_max, const scalar &z_max);
    Vector3 point_max;
    Vector3 point_min;
};

class Solver
{
public:
    Solver(Robo &manip);
    void SolveDirectKinematic(const Angles &q, Matrix4x4 TR[]);
    void SolveInverseKinematic(const scalar &accuracy, const StateVector &target, const Angles &curr_angle, Angles &res_angle);
    int CalcPolynomMaxQPP(const StateVector &start_point, const StateVector &end_point, const Angles &curr_angles, Angles& res_angle, const int &t_millisec, const scalar &dt_millisec, Polynom result[]);
    void CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const scalar &v_tcp, const scalar &a_tcp, const Angles &start_angles, Angles &res_angle, const ushort &dt_millisec, Polynom result[]);
//    int CalcPolynomCircle(const StateVector &start_point, const StateVector &end_point,)
    bool CheckTrajectoryForLinear(const Polynom &pol, const Vector3 &l1, const Vector3 &l2, const scalar &t_millisec, const scalar &max_delta);
    bool CheckTrajectoryForLinear(const Angles angles, const Vector3 &l1, const Vector3 &l2, const scalar &max_delta);
    bool MoveMeshes(const Matrix4x4 TR[]);;
    void CalcUMatrix()
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
                        U[k*6 + j] = U[k*6 + j] * T[ctr] * q;
                    else
                    {
                        U[k*6 + j] = U[k*6 + j] * T[ctr];
                    }
                }
            }
        }
    }
    bool SolveForwardDynamic(const Angles &angles)
    {
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

        scalar hikm_buff(0);
        Matrix4x4 u_buff;
        for (counter i = 0; i < 6; i++)
        {
            for (counter k = 0; k < 6; k++)
            {
                for (counter m = 0; m < 6; m++)
                {
                    hikm_buff = 0;
                    for (counter j = i; j < 6; j++)
                    {
                        u_buff.MakeIdentity();
                        for (counter ctr = 0; ctr < j; ctr++)
                        {
                            if (ctr == k && ctr == m)
                                u_buff = u_buff * q_sq * T[ctr];
                            else if (ctr == k || ctr == m)
                                u_buff = u_buff * q * T[ctr];
                            else
                                u_buff = u_buff * T[ctr];
                        }
                        hikm_buff += Trace(u_buff * m_ji[i].MultiplyTranspose(U[i*6 + j])) * angles.q_p[i] * angles.q_p[i];
                    }
                    m_moments.hikm[i] += hikm_buff;
                }
            }
        }
        for ( int i = 0; i < 6; i++)
        {
            m_moments.tao[i] = m_moments.p[i] + m_moments.dv[i] + m_moments.hikm[i];
            if (m_moments.tao[i] > 500)
                auto art = 0;
        }

        cout << "tao 1 = " << m_moments.tao[0] << "\ntao 2 = " << m_moments.tao[1] << "\ntao 3 = " << m_moments.tao[2] <<
        "\ntao 4 = " << m_moments.tao[3] << "\ntao 5 = " << m_moments.tao[4] << "\ntao 6 = " << m_moments.tao[5] << endl << endl;
        return true;
    };
//    void MoveMeshes(const Quaternion qtr[], const Vector3 v[]);
    pair<int, int> CheckCollisionAABB(const Matrix4x4 state[]);

private:
    Solver(Solver &solver) = delete;
    Solver(Solver &&solver) = delete;
    bool CheckAABB(const BoxAABB &box1, const BoxAABB &box2);
    void CalculateDeltas();
    const Matrix *manip_config;         //Матрица хранит конфигурацию манипулятора, не текущее состояние
    Matrix4x4 T[JOINTS_COUNT];          //buff matrix for calculations
    Emulator *m_emulator;
    std::vector<MeshCollider> m_meshes;
    std::vector<MeshCollider> m_meshes_buff;
    //Dynamic___________________
    Robo::Moments m_moments;
    array<Tensor,6> m_inertion;
    std::vector<scalar> m_mass;
    Matrix4x4 U[36];
    Vector m_gv{0, 0, -g, 0};
    array<rm::Vector, JOINTS_COUNT> m_cm;
    array<rm::Tensor, JOINTS_COUNT> m_i;
    array<rm::Matrix4x4, JOINTS_COUNT> m_ji;
    //__________________________
//    GJKCollision *collider;
//    Matrix4x4 T0[JOINTS_COUNT];
};

#endif // SOLVER_H
