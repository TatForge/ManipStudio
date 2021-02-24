#ifndef SOLVER_H
#define SOLVER_H
#include "robo.h"
#include "emulator.h"
//#include "my_math.h"

#define DELTA_T_FOR_LINE_CHECK 10 //дельта времени, через которую траектория проверяется на удаленность от прямой ms
#define DELTA_RANGE 0.0001   //дельта метров, на которые эффектор манипулятора может отклониться от прямой
#define MAX_KINEMATIC_ITERS 100
#define ACCURACY 10e-5

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
    Solver(Robo manip);
    void SolveDirectKinematic(const Angles &q, Matrix4x4 TR[]);
    int SolveInverseKinematic(const scalar &accuracy, const StateVector &target, const Angles &curr_angle, Angles &res_angle);
    int CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const Angles &start_angles, Angles &res_angle, const int &t_millisec, const scalar &dt_millisec, Polynom result[]);
    bool CheckTrajectoryForLinear(const Polynom &pol, const Vector3 &l1, const Vector3 &l2, const scalar &t_millisec, const scalar &max_delta);
    void CheckCollisionAABB(const Matrix4x4 state[]);

private:
    bool CheckAABB(const BoxAABB &box1, const BoxAABB &box2);
    void CalculateDeltas();
    const Matrix *manip_config;         //Матрица хранит конфигурацию манипулятора, не текущее состояние
    Matrix4x4 T[JOINTS_COUNT];          //buff matrix for calculations
    Emulator *emulator;
//    Matrix4x4 T0[JOINTS_COUNT];
};

#endif // SOLVER_H
