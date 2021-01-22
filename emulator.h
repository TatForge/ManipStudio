#ifndef EMULATOR_H
#define EMULATOR_H
#include "iostream"
#include "my_math.h"
#include "robo.h"
#include "thread"
#include "mainwindow.h"

#include <QApplication>
#include <qcustomplot.h>
//#include "QTimer"
//#include "QThread"
//#include "QObject"



enum ManipState {
        MOVE,
        STAND
};

using namespace RoboMath;

class Emulator/* : public QObject*/
{
//    Q_OBJECT
public:
    Emulator();
    Emulator(const Robo &manip);;
    ~Emulator();
    void MoveManip(Polynom *polynom_in, scalar moving_time);
    void SolveDirectKinematic();
    void SolveInverseKinematic();
    void OneIterMove();

//public slots:
//    void OneTick();
//    void startTimer()
//    {
//        m_timer.start(m_delta_t * 1000);
//    };
private:
//    QThread m_thread;
    Matrix4x4 T[JOINTS_COUNT];
    Matrix *state_matrix;
    Robo::Angles m_emu_angles;
    Polynom *m_polynom;
    Matrix4x4 L;
//    QTimer m_timer;
    scalar m_delta_t = 0.01;
    scalar t = 0;
    MainWindow w;
    void SetPolynomKF(Polynom *polynom_in);;
    void UpdateStateMatrix();
};

#endif // EMULATOR_H
