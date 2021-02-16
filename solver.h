#ifndef SOLVER_H
#define SOLVER_H
#include "robo.h"
#include "emulator.h"
//#include "my_math.h"

#define MAX_KINEMATIC_ITERS 20
#define ACCURACY 10e-6

using namespace rm;

class Solver
{
public:
    Solver(Robo manip);
    void SolveDirectKinematic(const Angles &q, Matrix4x4 T[]);
    int SolveInverseKinematic(const scalar &accuracy, const StateVector &target, const Angles &curr_angle, Angles &res_angle);
    int CalcPolynomLine(const StateVector &start_point, const StateVector &end_point, const Angles &start_angles, const int &t_millisec, const scalar &max_range, Polynom result[]);
private:
    void CalculateDeltas();
    const Matrix *manip_config;         //Матрица хранит конфигурацию манипулятора, не текущее состояние
    Matrix4x4 T[JOINTS_COUNT];          //buff matrix for calculations
    Emulator *emulator;
//    Matrix4x4 T0[JOINTS_COUNT];
};

#endif // SOLVER_H
