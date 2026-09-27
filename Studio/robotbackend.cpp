#include "robotbackend.h"

#include <QDir>
#include <QtMath>

#include <algorithm>

namespace
{
constexpr int kJoints = 6;
constexpr double kTrajectoryDtMs = 10;
constexpr int kPlayIntervalMs = 16;

QVariantList ToDegList(const QVector<double> &rad)
{
    QVariantList out;
    for (double v : rad)
        out.append(qRadiansToDegrees(v));
    return out;
}
}

RobotBackend::RobotBackend(const QString &data_dir, QObject *parent)
    : QObject(parent), m_data_dir(QDir(data_dir).absolutePath()), m_worker(new SolverWorker)
{
    // the core reads ManipulatorSettigns.ini and STLmodels/ relative to the working directory
    QDir::setCurrent(m_data_dir);
    m_target_rad.fill(0.0, kJoints);

    m_worker->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    connect(this, &RobotBackend::requestPose, m_worker, &SolverWorker::ComputePose);
    connect(this, &RobotBackend::requestMoveJ, m_worker, &SolverWorker::MoveJ);
    connect(m_worker, &SolverWorker::Initialized, this, &RobotBackend::OnInitialized);
    connect(m_worker, &SolverWorker::PoseReady, this, &RobotBackend::OnPose);
    connect(m_worker, &SolverWorker::TrajectoryReady, this, &RobotBackend::OnTrajectory);
    connect(m_worker, &SolverWorker::Failed, this, &RobotBackend::OnFailed);

    m_play_timer.setInterval(kPlayIntervalMs);
    connect(&m_play_timer, &QTimer::timeout, this, &RobotBackend::OnPlayTick);

    m_thread.setObjectName(QStringLiteral("solver"));
    m_thread.start();
    SetBusy(true);
    SetStatus(tr("Loading model from %1").arg(m_data_dir));
    QMetaObject::invokeMethod(m_worker, &SolverWorker::Initialize, Qt::QueuedConnection);
}

RobotBackend::~RobotBackend()
{
    m_thread.quit();
    m_thread.wait();
}

QVariantList RobotBackend::joints() const
{
    return ToDegList(m_target_rad);
}

QVariantList RobotBackend::tcp() const
{
    QVariantList out;
    for (double v : m_pose.tcp)
        out.append(v);
    return out;
}

QVariantList RobotBackend::frames() const
{
    QVariantList out;
    for (const QMatrix4x4 &m : m_pose.frames)
        out.append(QVariant::fromValue(m.column(3).toVector3D()));
    return out;
}

QVariantList RobotBackend::limits() const
{
    QVariantList out;
    for (int i = 0; i < m_lo.size(); i++)
        out.append(QVariantMap{{QStringLiteral("min"), qRadiansToDegrees(m_lo[i])}, {QStringLiteral("max"), qRadiansToDegrees(m_hi[i])}});
    return out;
}

double RobotBackend::progress() const
{
    const int n = sampleCount();
    return n > 1 ? double(m_sample) / (n - 1) : 0.0;
}

double RobotBackend::duration() const
{
    const int n = sampleCount();
    return n > 1 ? (n - 1) * m_dt_ms / 1000.0 : 0.0;
}

int RobotBackend::sampleCount() const
{
    return m_samples.size() / SolverWorker::SampleStride;
}

void RobotBackend::setJoint(int index, double deg)
{
    if (!m_ready || index < 0 || index >= kJoints)
        return;
    stop();
    m_target_rad[index] = std::clamp(qDegreesToRadians(deg), m_lo[index], m_hi[index]);
    emit poseChanged();
    emit requestPose(m_target_rad);
}

void RobotBackend::setJoints(const QVariantList &deg)
{
    if (!m_ready || deg.size() != kJoints)
        return;
    stop();
    for (int i = 0; i < kJoints; i++)
        m_target_rad[i] = std::clamp(qDegreesToRadians(deg[i].toDouble()), m_lo[i], m_hi[i]);
    emit poseChanged();
    emit requestPose(m_target_rad);
}

void RobotBackend::moveJ(const QVariantList &target_deg, double speed_deg_s, double accel_deg_s2)
{
    if (!m_ready || m_busy)
    {
        emit errorOccurred(tr("MoveJ: the solver is busy"));
        return;
    }
    if (target_deg.size() != kJoints)
    {
        emit errorOccurred(tr("MoveJ: expected %1 joint angles").arg(kJoints));
        return;
    }
    stop();
    QVector<double> target;
    for (const QVariant &v : target_deg)
        target.append(qDegreesToRadians(v.toDouble()));
    SetBusy(true);
    SetStatus(tr("Planning MoveJ..."));
    emit requestMoveJ(m_target_rad, target, qDegreesToRadians(speed_deg_s), qDegreesToRadians(accel_deg_s2), kTrajectoryDtMs);
}

void RobotBackend::stop()
{
    if (!m_play_timer.isActive())
        return;
    m_play_timer.stop();
    emit playingChanged();
}

QVariantList RobotBackend::zeroPose() const
{
    return {0, 0, 0, 0, 0, 0};
}

QVariantList RobotBackend::internalPose() const
{
    return {0, -45, 90, 45, 90, 0};
}

void RobotBackend::OnInitialized(const QVector<double> &lo, const QVector<double> &hi, double floor_z)
{
    m_lo = lo;
    m_hi = hi;
    m_ready = true;
    SetBusy(false);
    SetStatus(tr("Model loaded, floor at z = %1 m").arg(floor_z));
    emit readyChanged();
    emit requestPose(m_target_rad);
}

void RobotBackend::OnPose(const ArmPose &pose)
{
    m_pose = pose;
    emit poseChanged();
}

void RobotBackend::OnTrajectory(const QVector<float> &samples, double dt_ms)
{
    SetBusy(false);
    m_samples = samples;
    m_dt_ms = dt_ms;
    m_sample = 0;
    emit trajectoryChanged();
    SetStatus(tr("MoveJ: %1 samples, %2 s").arg(sampleCount()).arg(duration(), 0, 'f', 2));
    if (sampleCount() == 0)
        return;
    ShowSample(0);
    m_play_clock.start();
    m_play_timer.start();
    emit playingChanged();
}

void RobotBackend::OnFailed(const QString &what, const QString &message)
{
    SetBusy(false);
    SetStatus(tr("%1 failed").arg(what));
    emit errorOccurred(what + QStringLiteral(": ") + message);
}

void RobotBackend::OnPlayTick()
{
    const int last = sampleCount() - 1;
    const int index = std::min(last, int(m_play_clock.elapsed() / m_dt_ms));
    ShowSample(index);
    if (index == last)
    {
        stop();
        emit motionFinished();
    }
}

void RobotBackend::ShowSample(int index)
{
    m_sample = index;
    const float *s = m_samples.constData() + index * SolverWorker::SampleStride;
    for (int i = 0; i < kJoints; i++)
        m_target_rad[i] = s[i];
    emit progressChanged();
    emit poseChanged();
    emit requestPose(m_target_rad);
}

void RobotBackend::SetBusy(bool busy)
{
    if (m_busy == busy)
        return;
    m_busy = busy;
    emit busyChanged();
}

void RobotBackend::SetStatus(const QString &status)
{
    if (m_status == status)
        return;
    m_status = status;
    emit statusChanged();
}
