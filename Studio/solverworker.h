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

// Result of planning one motion. On failure `error` is set, `samples` and
// `tcp_path` are empty and `moments` holds the samples evaluated before the
// stop (the last one is where a motor limit was exceeded, if that was the cause).
struct PlannedTrajectory
{
    QVector<float> samples;             // SampleStride floats per sample (q, q', q'' in rad)
    QVector<float> moments;             // 6 joint moments (N*m) per evaluated sample
    QVector<QVector3D> tcp_path;        // flange origin (m) of every sample
    double dt_ms = 0;
    QString error;
};
Q_DECLARE_METATYPE(PlannedTrajectory)

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
    void Initialized(const QVector<double> &limits_min_rad, const QVector<double> &limits_max_rad, double floor_z,
                     const QVector<double> &max_moments);
    void PoseReady(const ArmPose &pose);
    void TrajectoryReady(const PlannedTrajectory &trajectory);
    void Failed(const QString &what, const QString &message);

private:
    std::unique_ptr<Robo> m_robot;
    std::unique_ptr<Solver> m_solver;
};

#endif // SOLVERWORKER_H
