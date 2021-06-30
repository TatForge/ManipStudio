#ifndef EMULATOR_H
#define EMULATOR_H
#include "iostream"
#include "my_math.h"
#include "robo.h"
#include "thread"
#include "mutex"
#include "sharedmem.h"
//#include "sysvsharedmemory.h"

#if WITH_WINDOW
#include "mainwindow.h"
#endif

#define VREMENNO_DLYA_VISUALKI

#ifdef VREMENNO_DLYA_VISUALKI
#include "meshcollider.h"
#endif

typedef SharedMem SharedMemory;

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
    Emulator(const Robo &manip);
    ~Emulator();
    void OneIterMoveLinear(const Angles &angle1, const Angles &angle2, const int &dt_msec, bool is_last, Vector3 accel);
    void MoveManipLinear(const Angles angles[], size_t size, scalar dt_msec, const Vector3 accel[]);
    void MoveManipLinear(std::vector<Angles> angles, std::vector<Vector3> speed, scalar dt_msec);
    void MoveManipPolynom(Polynom polynom_arr[], scalar moving_time, scalar dt);
    void SendDataToShared(SharedMemKey skey, float *data_ptr, size_t size);
    void MoveManipPolynom(Polynom polynom_in, int moving_time_millisec);
    void OneIterMove(bool is_last);
    void FillManipPositionData();
    void SolveDirectKinematic();
    void MoveMeshes();

    int iter;
#if WITH_WINDOW
    MainWindow moments_window;
#endif
private:
    void SetPolynomKF(Polynom polynom_in);
    void UpdateStateMatrix();

    float points_to_visual[999];
    float meshes_to_visual[999];
    float manip_to_visual[18];
    short m_delta_t_millisec = 1;
    SharedMemory *shared_points;
    SharedMemory *shared_meshes;
    SharedMemory *shared_manip;
    mutex data_to_shared_mutex;
    Matrix4x4 T[JOINTS_COUNT];
    std::thread thread_points;
    scalar m_delta_t = 0.001;
    std::thread thread_manip;
    rm::Angles m_emu_angles;
    Matrix *state_matrix;
    Polynom m_polynom;
    Matrix4x4 L;
    int t_m = 0;
#ifdef VREMENNO_DLYA_VISUALKI
    std::vector<MeshCollider> meshes;
    std::vector<MeshCollider> meshes_buff;
#endif

#if WITH_WINDOW
    MainWindow w;
#endif
};

#endif // EMULATOR_H
