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
    qRegisterMetaType<PlannedTrajectory>();
    qRegisterMetaType<std::vector<ArmPart>>();
    qRegisterMetaType<RobotSettings>();
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
        EmitState(false);
    });
}

void SolverWorker::EmitState(bool saved)
{
    QVector<double> lo, hi;
    for (const auto &l : m_solver->GetJointLimits())
    {
        lo.append(l.first);
        hi.append(l.second);
    }
    QVector<double> max_moments;
    for (scalar m : m_robot->GetMaxJointMoments())
        max_moments.append(m);

    RobotSettings s;
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        s.dh[i] = m_robot->GetDH(i);
        s.limit_min[i] = lo[i];
        s.limit_max[i] = hi[i];
        s.max_moments[i] = max_moments[i];
        s.joint_radii[i] = m_robot->GetJointRadii()[i];
    }
    s.floor_z = m_robot->GetFloorHeight();
    s.links = m_robot->GetLinkDynamics();
    s.payload = m_robot->GetPayload();
    emit SettingsReady(s, saved);
    emit Initialized(lo, hi, m_solver->GetFloorHeight(), max_moments, m_solver->GetArmParts());
}

void SolverWorker::ApplySettings(const RobotSettings &s, bool save)
{
    if (!m_robot)
    {
        emit Failed(QStringLiteral("settings"), QStringLiteral("the model is not loaded"));
        return;
    }
    Guarded(this, QStringLiteral("settings"), [&] {
        // check everything first, so a rejected set leaves the model untouched
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            const std::string joint = "joint " + std::to_string(i + 1);
            if (!(s.limit_min[i] < s.limit_max[i]))
                throw std::invalid_argument(joint + ": the lower limit must be below the upper one");
            if (!(s.max_moments[i] > 0))
                throw std::invalid_argument(joint + ": the max moment must be positive");
            if (!(s.joint_radii[i] > 0))
                throw std::invalid_argument(joint + ": the radius must be positive");
            if (!(s.links[i].mass >= 0))
                throw std::invalid_argument("link " + std::to_string(i + 1) + ": the mass must not be negative");
            if (!std::isfinite(s.dh[i].a) || !std::isfinite(s.dh[i].alpha) || !std::isfinite(s.dh[i].d))
                throw std::invalid_argument(joint + ": DH parameters must be numbers");
        }
        if (!(s.payload.mass >= 0))
            throw std::invalid_argument("the payload mass must not be negative");
        if (!std::isfinite(s.floor_z))
            throw std::invalid_argument("the floor height must be a number");

        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            m_robot->SetDH(i, s.dh[i]);
            m_robot->SetJointLimit(i, s.limit_min[i], s.limit_max[i]);
            m_robot->SetMaxJointMoment(i, s.max_moments[i]);
            m_robot->SetLinkDynamics(i, s.links[i]);
            m_robot->SetJointRadius(i, s.joint_radii[i]);
        }
        m_robot->SetFloorHeight(s.floor_z);
        m_robot->SetPayload(s.payload);
        if (save)
            m_robot->WriteSettingsToFile();
        m_solver.reset();       // its emulator owns the shared memory segments
        m_solver = std::make_unique<Solver>(*m_robot);
        EmitState(save);
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
    PlannedTrajectory out;
    out.dt_ms = dt_ms;
    std::vector<float> results;
    try
    {
        m_solver->MoveJ(ToAngles(from_rad), ToAngles(to_rad), float(speed_rad_s), float(accel_rad_s2), float(dt_ms), results);
    }
    catch (const std::exception &e)
    {
        out.error = QString::fromStdString(e.what());
    }
    catch (const std::string &s)
    {
        out.error = QString::fromStdString(s);
    }
    catch (...)
    {
        out.error = QStringLiteral("unknown error");
    }
    for (const Robo::Moments &m : m_solver->GetTrajectoryMoments())
        for (int i = 0; i < JOINTS_COUNT; i++)
            out.moments.append(float(m.tao[i]));
    if (!out.error.isEmpty())
    {
        emit TrajectoryReady(out);
        return;
    }
    out.samples = QVector<float>(results.begin(), results.end());
    Matrix4x4 TR[JOINTS_COUNT];
    for (size_t k = 0; k + SampleStride <= results.size(); k += SampleStride)
    {
        Angles a;
        for (int i = 0; i < JOINTS_COUNT; i++)
            a.q[i] = results[k + i];
        m_solver->SolveDirectKinematic(a, TR);
        const Matrix4x4 &t = TR[JOINTS_COUNT - 1];
        out.tcp_path.append(QVector3D(float(t.at(0, 3)), float(t.at(1, 3)), float(t.at(2, 3))));
    }
    emit TrajectoryReady(out);
}
