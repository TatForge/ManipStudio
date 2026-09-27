#ifndef SOLVERWORKER_H
#define SOLVERWORKER_H

#include <QMatrix4x4>
#include <QObject>
#include <QString>
#include <QVector3D>
#include <QVector>

#include <memory>

class Robo;
class Solver;

// Pose of the arm for one set of joint angles.
struct ArmPose
{
    QVector<double> joints_rad;         // 6 joint angles
    QVector<QMatrix4x4> frames;         // base -> joint i frame (TR[i] of the solver), metres
    QVector<double> tcp;                // x, y, z (m), roll, pitch, yaw (deg), ZYX convention
};
Q_DECLARE_METATYPE(ArmPose)

// Owns the robot model and the solver. Lives in its own thread: every call
// into the core goes through the queued slots below, so the solver is never
// touched from two threads.
class SolverWorker : public QObject
{
    Q_OBJECT
public:
    // Samples of a trajectory: 18 floats per sample (q, q', q'' for 6 joints, rad).
    static constexpr int SampleStride = 18;

    explicit SolverWorker(QObject *parent = nullptr);
    ~SolverWorker() override;

public slots:
    // Loads ManipulatorSettigns.ini and STLmodels/ from the current directory.
    void Initialize();
    void ComputePose(const QVector<double> &joints_rad);
    void MoveJ(const QVector<double> &from_rad, const QVector<double> &to_rad, double speed_rad_s, double accel_rad_s2, double dt_ms);

signals:
    void Initialized(const QVector<double> &limits_min_rad, const QVector<double> &limits_max_rad, double floor_z);
    void PoseReady(const ArmPose &pose);
    // tcp_path: flange origin (m) of every sample
    void TrajectoryReady(const QVector<float> &samples, double dt_ms, const QVector<QVector3D> &tcp_path);
    void Failed(const QString &what, const QString &message);

private:
    std::unique_ptr<Robo> m_robot;
    std::unique_ptr<Solver> m_solver;
};

#endif // SOLVERWORKER_H
