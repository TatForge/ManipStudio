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
    qRegisterMetaType<MotionStep>();
    qRegisterMetaType<QVector<MotionStep>>();
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

std::array<double, 6> SolverWorker::TcpPose(const Angles &a)
{
    Matrix4x4 TR[JOINTS_COUNT];
    m_solver->SolveDirectKinematic(a, TR);
    const Matrix4x4 &t = TR[JOINTS_COUNT - 1];
    return {t.at(0, 3), t.at(1, 3), t.at(2, 3), std::atan2(t.at(2, 1), t.at(2, 2)),
            std::atan2(-t.at(2, 0), std::hypot(t.at(2, 1), t.at(2, 2))), std::atan2(t.at(1, 0), t.at(0, 0))};
}

namespace
{
StateVector ToState(const std::array<double, 6> &p)
{
    return StateVector(double(p[0]), double(p[1]), double(p[2]), double(p[3]), double(p[4]), double(p[5]));
}

const char *KindName(MotionStep::Kind kind)
{
    switch (kind)
    {
    case MotionStep::Joint: return "MoveJ";
    case MotionStep::Linear: return "MoveL";
    case MotionStep::Circular: return "MoveC";
    }
    return "";
}
}

void SolverWorker::Plan(const QVector<double> &start_rad, const QVector<MotionStep> &steps, double dt_ms)
{
    PlannedTrajectory out;
    out.dt_ms = dt_ms;
    if (!m_solver)
    {
        out.error = QStringLiteral("solver is not initialized");
        emit TrajectoryReady(out);
        return;
    }
    const bool program = steps.size() > 1;
    Angles current = ToAngles(start_rad);
    QVector<float> samples, moments;
    for (int i = 0; i < steps.size(); i++)
    {
        const MotionStep &step = steps[i];
        std::vector<float> results;
        QString error;
        try
        {
            switch (step.kind)
            {
            case MotionStep::Joint:
            {
                Angles target;
                bool still = true;
                for (int j = 0; j < JOINTS_COUNT; j++)
                {
                    target.q[j] = step.joints[j];
                    still = still && std::abs(target.q[j] - current.q[j]) < 1e-9;
                }
                if (program && still)
                    continue;
                m_solver->MoveJ(current, target, float(step.speed), float(step.accel), float(dt_ms), results);
                break;
            }
            case MotionStep::Linear:
            case MotionStep::Circular:
            {
                const std::array<double, 6> from = TcpPose(current);
                std::array<double, 6> to = step.pose;
                // the same orientation can be written with angles 360 deg apart: take the short way
                for (int k = 3; k < 6; k++)
                    to[k] = from[k] + std::remainder(to[k] - from[k], 2 * M_PI);
                bool still = true;
                for (int k = 0; k < 6; k++)
                    still = still && std::abs(to[k] - from[k]) < 1e-9;
                if (program && still && step.kind == MotionStep::Linear)
                    continue;
                if (step.kind == MotionStep::Linear)
                    m_solver->MoveL(ToState(from), ToState(to), step.speed, step.accel, current, ushort(dt_ms), results);
                else
                    m_solver->MoveC(current, ToState(from), ToState({step.mid[0], step.mid[1], step.mid[2], to[3], to[4], to[5]}),
                                    ToState(to), step.speed, step.accel, dt_ms, results);
                break;
            }
            }
        }
        catch (const std::exception &e)
        {
            error = QString::fromStdString(e.what());
        }
        catch (const std::string &e)
        {
            error = QString::fromStdString(e);
        }
        catch (const QString &e)
        {
            error = e;
        }
        catch (...)
        {
            error = QStringLiteral("unknown error");
        }

        // a step after the first starts where the previous one ended: drop the repeated sample
        const int skip = samples.isEmpty() ? 0 : 1;
        const std::vector<Robo::Moments> step_moments = m_solver->GetTrajectoryMoments();
        for (size_t k = skip; k < step_moments.size(); k++)
            for (int j = 0; j < JOINTS_COUNT; j++)
                moments.append(float(step_moments[k].tao[j]));
        if (!error.isEmpty())
        {
            out.error = program ? QStringLiteral("step %1 (%2): %3").arg(i + 1).arg(QLatin1String(KindName(step.kind)), error) : error;
            out.moments = moments;
            emit TrajectoryReady(out);
            return;
        }
        for (size_t k = size_t(skip) * SampleStride; k < results.size(); k++)
            samples.append(results[k]);
        if (results.size() >= size_t(SampleStride))
            for (int j = 0; j < JOINTS_COUNT; j++)
                current.q[j] = results[results.size() - SampleStride + j];
    }
    if (samples.isEmpty())
    {
        out.error = QStringLiteral("the program does not move the arm");
        emit TrajectoryReady(out);
        return;
    }
    out.samples = samples;
    out.moments = moments;
    Matrix4x4 TR[JOINTS_COUNT];
    for (int k = 0; k + SampleStride <= samples.size(); k += SampleStride)
    {
        Angles a;
        for (int j = 0; j < JOINTS_COUNT; j++)
            a.q[j] = samples[k + j];
        m_solver->SolveDirectKinematic(a, TR);
        const Matrix4x4 &t = TR[JOINTS_COUNT - 1];
        out.tcp_path.append(QVector3D(float(t.at(0, 3)), float(t.at(1, 3)), float(t.at(2, 3))));
    }
    emit TrajectoryReady(out);
}
