#ifndef EMULATOR_H
#define EMULATOR_H
#include "iostream"
#include "my_math.h"
#include "robo.h"
#include "thread"
#include "mutex"
#include "mainwindow.h"
#include "sharedmem.h"


#include <QApplication>
//#include <qcustomplot.h>

#define VREMENNO_DLYA_VISUALKI

#ifdef VREMENNO_DLYA_VISUALKI
#include "meshcollider.h"
#endif

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
    POINTS,
    MESHES
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
    void MoveManipPolynom(Polynom polynom_in, int moving_time_millisec);
    void MoveManipPolynom(Polynom polynom_arr[], scalar moving_time, scalar dt);
    void MoveManipLinear(const Angles angles[], size_t size, scalar dt_msec, const Vector3 accel[]);
    void MoveManipLinear(std::vector<Angles> angles, std::vector<Vector3> accel, scalar dt_msec);
    void SolveDirectKinematic();
    void OneIterMove(bool is_last);
    void OneIterMoveLinear(const Angles &angle1, const Angles &angle2, const int &dt_msec, bool is_last, Vector3 accel);
    void SendDataToShared(SharedMemKey skey, float *data_ptr, size_t size);
    void FillManipPositionData();
    void MoveThatShit();
    MainWindow moments_window;
private:
    Matrix4x4 T[JOINTS_COUNT];
    Matrix *state_matrix;
    rm::Angles m_emu_angles;
    Polynom m_polynom;
    Matrix4x4 L;
    SharedMem *shared_manip;
    SharedMem *shared_points;
    SharedMem *shared_meshes;
    float manip_to_visual[18];
    float points_to_visual[999];
    float meshes_to_visual[999];
    scalar m_delta_t = 0.001;
    short m_delta_t_millisec = 1;
    int t_m = 0;
    mutex data_to_shared_mutex;
    std::thread thread_points;
    std::thread thread_manip;
#ifdef VREMENNO_DLYA_VISUALKI
    std::vector<MeshCollider> meshes;
    std::vector<MeshCollider> meshes_buff;
#endif

//#if !BENCHMARK
    MainWindow w;
//#else
//    class EmulatorConsoleOut
//    {
//    public:
//        void PlotCoordinates(double value_x, double value_y, double value_z, double yaw, double pitch, double roll, double t, bool is_last = false){};
//        void PlotSpeeds(rm::Angles speeds, double t, bool is_last = false){};
//        void PrintResult(double x, double y, double z, double yaw, double pitch, double roll, double t){};
//    };
//    EmulatorConsoleOut w;
//#endif
    void SetPolynomKF(Polynom polynom_in);;
    void UpdateStateMatrix();
};

#endif // EMULATOR_H
