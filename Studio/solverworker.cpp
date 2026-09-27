#include "solverworker.h"

#include "robo.h"
#include "solver.h"

#include <cmath>
#include <exception>
#include <string>

namespace
{
Angles ToAngles(const QVector<double> &q)
{
    Angles a;
    for (int i = 0; i < JOINTS_COUNT && i < q.size(); i++)
        a.q[i] = q[i];
    return a;
}

QMatrix4x4 ToQMatrix(const Matrix4x4 &m)
{
    QMatrix4x4 r;
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            r(row, col) = float(m.at(row, col));
    return r;
}

// Runs a call into the core and reports any exception type it may throw.
template <typename F>
bool Guarded(SolverWorker *w, const QString &what, F &&f)
{
    QString message;
    try
    {
        f();
        return true;
    }
    catch (const std::exception &e)
    {
        message = QString::fromStdString(e.what());
    }
    catch (const std::string &s)
    {
        message = QString::fromStdString(s);
    }
    catch (...)
    {
        message = QStringLiteral("unknown error");
    }
    emit w->Failed(what, message);
    return false;
}
}

SolverWorker::SolverWorker(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<ArmPose>();
}

SolverWorker::~SolverWorker() = default;

void SolverWorker::Initialize()
{
    Guarded(this, QStringLiteral("load"), [this] {
        auto robot = std::make_unique<Robo>(JOINTS_COUNT);
        if (!robot->LoadManip())
            throw std::runtime_error("cannot read ManipulatorSettigns.ini");
        m_solver = std::make_unique<Solver>(*robot);
        m_robot = std::move(robot);
        QVector<double> lo, hi;
        for (const auto &l : m_solver->GetJointLimits())
        {
            lo.append(l.first);
            hi.append(l.second);
        }
        emit Initialized(lo, hi, m_solver->GetFloorHeight());
    });
}

void SolverWorker::ComputePose(const QVector<double> &joints_rad)
{
    if (!m_solver)
        return;
    Matrix4x4 TR[JOINTS_COUNT];
    m_solver->SolveDirectKinematic(ToAngles(joints_rad), TR);
    ArmPose pose;
    pose.joints_rad = joints_rad;
    for (const auto &m : TR)
        pose.frames.append(ToQMatrix(m));
    const Matrix4x4 &t = TR[JOINTS_COUNT - 1];
    const double roll = std::atan2(t.at(2, 1), t.at(2, 2));
    const double pitch = std::atan2(-t.at(2, 0), std::hypot(t.at(2, 1), t.at(2, 2)));
    const double yaw = std::atan2(t.at(1, 0), t.at(0, 0));
    pose.tcp = {t.at(0, 3), t.at(1, 3), t.at(2, 3), ToDeg(roll), ToDeg(pitch), ToDeg(yaw)};
    emit PoseReady(pose);
}

void SolverWorker::MoveJ(const QVector<double> &from_rad, const QVector<double> &to_rad, double speed_rad_s, double accel_rad_s2,
                         double dt_ms)
{
    if (!m_solver)
    {
        emit Failed(QStringLiteral("MoveJ"), QStringLiteral("solver is not initialized"));
        return;
    }
    std::vector<float> results;
    if (!Guarded(this, QStringLiteral("MoveJ"), [&] {
            m_solver->MoveJ(ToAngles(from_rad), ToAngles(to_rad), float(speed_rad_s), float(accel_rad_s2), float(dt_ms), results);
        }))
        return;
    QVector<QVector3D> path;
    Matrix4x4 TR[JOINTS_COUNT];
    for (size_t k = 0; k + SampleStride <= results.size(); k += SampleStride)
    {
        Angles a;
        for (int i = 0; i < JOINTS_COUNT; i++)
            a.q[i] = results[k + i];
        m_solver->SolveDirectKinematic(a, TR);
        const Matrix4x4 &t = TR[JOINTS_COUNT - 1];
        path.append(QVector3D(float(t.at(0, 3)), float(t.at(1, 3)), float(t.at(2, 3))));
    }
    emit TrajectoryReady(QVector<float>(results.begin(), results.end()), dt_ms, path);
}
