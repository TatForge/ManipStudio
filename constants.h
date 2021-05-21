#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

//______ALL_PROJECT_CONSTANTS______
#define WITH_BENCHMARK  0
#define WITH_GTESTS     0
#define WITH_QTESTS     0
#define WITH_COLLISIONS 1
#define MULTITHREAD     0
#define WITH_WINDOW     0
//assertions turn off
//#define NDEBUG
#define SMESH +0.65
#define DT_MILLISEC 10
//__________________________________
#define RANGE_TO_PLOT 0.025  // 25 mm

#define RAD (M_PI/180)
#define DEG (180/M_PI)
#define JOINTS_COUNT 6

//____Math_constants________________
#define DOUBLE_SCALAR
//#define DEBUG_COUT

#ifdef DOUBLE_SCALAR
typedef double scalar;
#else
typedef float scalar;
#endif
const scalar g = 9.80665;
inline scalar ToRad(scalar degrees) {return degrees * RAD;}
inline scalar ToDeg(scalar radians) {return radians * DEG;}
inline scalar ToSec(scalar milliseconds) {return milliseconds / 1000.0;}
inline scalar ToMillisec(scalar seconds) {return floor(seconds * 1000.0 + 0.5);}
//_____________________________

//______Robo___constants_______
#define POSITION_AND_ROTATION
//#define CALC_COUT
#define JOINTS_COUNT 6

#ifdef POSITION_AND_ROTATION
#define EFFECTOR_DEGREE_OF_FREEDOM 6
#else
#define EFFECTOR_DEGREE_OF_FREEDOM 3
#endif
const scalar damping_constant = 0.1;
const scalar damping_constant_sq = damping_constant * damping_constant;
const scalar MaxAngleDls = ToRad(45.0);
//_____________________________
//______Solver constants_____          //Включение BENCHMARK отключает показ окна для эмулятора
#define DELTA_T_FOR_LINE_CHECK 10  //дельта времени, через которую траектория проверяется на удаленность от прямой ms
#define DELTA_RANGE 0.0001         //дельта метров, на которые эффектор манипулятора может отклониться от прямой
#define MAX_KINEMATIC_ITERS 100
#define ACCURACY 10e-8

//_____________________________




#endif // CONSTANTS_H
