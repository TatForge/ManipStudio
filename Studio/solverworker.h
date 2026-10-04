#ifndef SOLVERWORKER_H
#define SOLVERWORKER_H

#include <QMatrix4x4>
#include <QObject>
#include <QString>
#include <QVector3D>
#include <QVector>

#include "armshape.h"
#include "robo.h"

#include <memory>
#include <vector>

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
    // slowed down by a joint speed / acceleration limit: the slowest step's planned / requested
    // speed, its joint (-1: not slowed) and step index
    double speed_scale = 1;
    int limit_joint = -1;
    bool limit_by_accel = false;
    int limit_step = -1;
};
Q_DECLARE_METATYPE(PlannedTrajectory)
Q_DECLARE_METATYPE(std::vector<ArmPart>)

// Everything the settings editor changes (SI units, radians).
struct RobotSettings
{
    std::array<Robo::DHParams, JOINTS_COUNT> dh;
    std::array<double, JOINTS_COUNT> limit_min, limit_max;
    std::array<double, JOINTS_COUNT> max_moments;
    std::array<double, JOINTS_COUNT> max_speeds, max_accels;      // rad/s, rad/s^2
    double floor_z = 0;
    std::array<Robo::LinkDynamics, JOINTS_COUNT> links;
    Robo::Payload payload;
    std::array<double, JOINTS_COUNT> joint_radii;
};
Q_DECLARE_METATYPE(RobotSettings)

// One step of a motion: a synchronized joint move (J), a straight TCP line (L)
// or a circular arc through `mid` (C). SI units, radians.
struct MotionStep
{
    enum Kind { Joint, Linear, Circular };
    Kind kind = Joint;
    std::array<double, JOINTS_COUNT> joints{};      // J: target angles
    std::array<double, 6> pose{};                   // L, C: end pose x, y, z, roll, pitch, yaw (ZYX)
    std::array<double, 3> mid{};                    // C: point on the arc
    double speed = 0;                               // J: rad/s; L, C: m/s (rad/s for a pure turn)
    double accel = 0;                               // J: rad/s^2; L, C: m/s^2
};
Q_DECLARE_METATYPE(MotionStep)
Q_DECLARE_METATYPE(QVector<MotionStep>)

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
    // Plans the steps one after another from `start` into one trajectory. A step that
    // fails ends the plan; with several steps, steps that would not move are skipped.
    void Plan(const QVector<double> &start_rad, const QVector<MotionStep> &steps, double dt_ms);
    // validates, applies to the model, optionally writes the settings file, rebuilds the solver
    void ApplySettings(const RobotSettings &settings, bool save);

signals:
    void Initialized(const QVector<double> &limits_min_rad, const QVector<double> &limits_max_rad, double floor_z,
                     const QVector<double> &max_moments, const std::vector<ArmPart> &shape);
    void PoseReady(const ArmPose &pose);
    void SettingsReady(const RobotSettings &settings, bool saved);
    void TrajectoryReady(const PlannedTrajectory &trajectory);
    void Failed(const QString &what, const QString &message);

private:
    void EmitState(bool saved);
    // TCP pose x, y, z, roll, pitch, yaw (ZYX, rad) for joint angles
    std::array<double, 6> TcpPose(const RoboMath::Angles &a);

    std::unique_ptr<Robo> m_robot;
    std::unique_ptr<Solver> m_solver;
};

#endif // SOLVERWORKER_H
