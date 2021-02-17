#ifndef EMULATOR_H
#define EMULATOR_H
#include "iostream"
#include "my_math.h"
#include "robo.h"
#include "thread"
#include "mainwindow.h"
#include "sharedmem.h"

#include <QApplication>
#include <qcustomplot.h>

#define RANGE_TO_PLOT 0.01 // 25 mm

//#include "QTimer"
//#include "QThread"
//#include "QObject"



enum ManipState {
        MOVE,
        STAND
};
enum SharedMemKey
{
    MANIP = 1,
    POINTS
};

using namespace RoboMath;

class Emulator
{
public:
    int iter;
public:
    Emulator();
    Emulator(const Robo &manip);
    ~Emulator();
    void MoveManip(Polynom polynom_in, int moving_time_millisec);
    void MoveManip(Polynom polynom_arr[], scalar moving_time, scalar dt);
    void SolveDirectKinematic();
    void OneIterMove(bool is_last);
    void SendDataToShared(SharedMemKey skey, float *data_ptr, size_t size);
    void FillManipPositionData();
private:
    Matrix4x4 T[JOINTS_COUNT];
    Matrix *state_matrix;
    rm::Angles m_emu_angles;
    Polynom m_polynom;
    Matrix4x4 L;
    SharedMem *shared_manip;
    SharedMem *shared_points;
    float manip_to_visual[18];
    float points_to_visual[999];
    scalar m_delta_t = 0.001;
    scalar t = 0;
    MainWindow w;
    void SetPolynomKF(Polynom polynom_in);;
    void UpdateStateMatrix();
};

#endif // EMULATOR_H
