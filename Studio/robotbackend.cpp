#include "robotbackend.h"

#include "meshlayout.h"

#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QXYSeries>
#include <QQuaternion>
#include <QUrl>
#include <QtMath>

#include <algorithm>

namespace
{
constexpr int kJoints = 6;
constexpr double kTrajectoryDtMs = 10;
constexpr int kPlayIntervalMs = 16;

QVariantMap ToNodePose(const QMatrix4x4 &m)
{
    return {{QStringLiteral("position"), QVariant::fromValue(m.column(3).toVector3D())},
            {QStringLiteral("rotation"), QVariant::fromValue(QQuaternion::fromRotationMatrix(m.toGenericMatrix<3, 3>()))}};
}

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

QVariantList RobotBackend::meshParts() const
{
    QVariantList out;
    for (const MeshLayout::Part &p : MeshLayout::Parts())
        out.append(QVariantMap{{QStringLiteral("name"), p.name},
                               {QStringLiteral("source"), QUrl::fromLocalFile(QDir(m_data_dir).filePath(p.file))},
                               {QStringLiteral("obstacle"), p.obstacle}});
    return out;
}

QVariantList RobotBackend::meshPoses() const
{
    QVariantList out;
    for (const QMatrix4x4 &m : MeshLayout::Placements(m_pose.frames))
        out.append(ToNodePose(m));
    return out;
}

QVariantMap RobotBackend::toolPose() const
{
    return m_pose.frames.isEmpty() ? QVariantMap() : ToNodePose(m_pose.frames.last());
}

QVariantList RobotBackend::tcpPath() const
{
    QVariantList out;
    for (const QVector3D &p : m_tcp_path)
        out.append(QVariant::fromValue(p));
    return out;
}

QVariantList RobotBackend::maxMoments() const
{
    QVariantList out;
    for (double m : m_max_moments)
        out.append(m);
    return out;
}

QVariantList RobotBackend::peaks() const
{
    QVariantList out;
    const int n = momentCount();
    if (n == 0)
        return out;
    for (int j = 0; j < kJoints; j++)
    {
        int at = 0;
        double peak = 0;
        for (int k = 0; k < n; k++)
        {
            const double v = std::abs(m_moments[k * kJoints + j]);
            if (v > peak)
            {
                peak = v;
                at = k;
            }
        }
        const double limit = j < m_max_moments.size() ? m_max_moments[j] : 0.0;
        out.append(QVariantMap{{QStringLiteral("peak"), peak},
                               {QStringLiteral("time"), at * m_dt_ms / 1000.0},
                               {QStringLiteral("limit"), limit},
                               {QStringLiteral("ratio"), limit > 0 ? peak / limit : 0.0},
                               {QStringLiteral("exceeded"), limit > 0 && peak > limit}});
    }
    return out;
}

QList<QPointF> RobotBackend::Series(const QString &kind, int joint) const
{
    QList<QPointF> out;
    if (joint < 0 || joint >= kJoints)
        return out;
    const double dt = m_dt_ms / 1000.0;
    if (kind == QLatin1String("moment"))
    {
        for (int k = 0; k < momentCount(); k++)
            out.append({k * dt, m_moments[k * kJoints + joint]});
        return out;
    }
    // each sample stores q, q', q'' as consecutive blocks of six
    int block = -1;
    if (kind == QLatin1String("position"))
        block = 0;
    else if (kind == QLatin1String("velocity"))
        block = 1;
    else if (kind == QLatin1String("acceleration"))
        block = 2;
    if (block < 0)
        return out;
    for (int k = 0; k < sampleCount(); k++)
        out.append({k * dt, qRadiansToDegrees(double(m_samples[k * SolverWorker::SampleStride + block * kJoints + joint]))});
    return out;
}

QVariantMap RobotBackend::seriesRange(const QString &kind) const
{
    double lo = 0, hi = 0, x = 0;
    for (int j = 0; j < kJoints; j++)
        for (const QPointF &p : Series(kind, j))
        {
            lo = std::min(lo, p.y());
            hi = std::max(hi, p.y());
            x = std::max(x, p.x());
        }
    return {{QStringLiteral("xMax"), x}, {QStringLiteral("yMin"), lo}, {QStringLiteral("yMax"), hi}};
}

bool RobotBackend::fillSeries(QObject *series, const QString &kind, int joint) const
{
    auto *xy = qobject_cast<QXYSeries *>(series);
    if (!xy)
        return false;
    xy->replace(Series(kind, joint));
    return true;
}

bool RobotBackend::exportCsv(const QUrl &url) const
{
    const int n = sampleCount();
    QFile file(url.isLocalFile() ? url.toLocalFile() : url.toString());
    if (n == 0 || !file.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    QTextStream out(&file);
    out << "t_s";
    const char *const columns[3][2] = {{"q", "_deg"}, {"dq", "_deg_s"}, {"ddq", "_deg_s2"}};
    for (const auto &c : columns)
        for (int j = 1; j <= kJoints; j++)
            out << ',' << c[0] << j << c[1];
    for (int j = 1; j <= kJoints; j++)
        out << ",tau" << j << "_Nm";
    out << '\n';
    for (int k = 0; k < n; k++)
    {
        out << QString::number(k * m_dt_ms / 1000.0, 'f', 3);
        const float *s = m_samples.constData() + k * SolverWorker::SampleStride;
        for (int i = 0; i < 3 * kJoints; i++)
            out << ',' << QString::number(qRadiansToDegrees(double(s[i])), 'g', 8);
        for (int j = 0; j < kJoints; j++)
            out << ',' << (k < momentCount() ? QString::number(m_moments[k * kJoints + j], 'g', 8) : QString());
        out << '\n';
    }
    return true;
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

void RobotBackend::play()
{
    const int n = sampleCount();
    if (n == 0 || m_busy)
        return;
    stop();
    if (m_sample >= n - 1)
        ShowSample(0);
    m_play_from = m_sample;
    m_play_clock.start();
    m_play_timer.start();
    emit playingChanged();
}

void RobotBackend::seek(double seconds)
{
    const int n = sampleCount();
    if (n == 0)
        return;
    stop();
    ShowSample(std::clamp(int(std::lround(seconds * 1000.0 / m_dt_ms)), 0, n - 1));
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

void RobotBackend::OnInitialized(const QVector<double> &lo, const QVector<double> &hi, double floor_z, const QVector<double> &max_moments)
{
    m_max_moments = max_moments;
    m_lo = lo;
    m_hi = hi;
    m_floor_z = floor_z;
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

void RobotBackend::OnTrajectory(const PlannedTrajectory &trajectory)
{
    SetBusy(false);
    m_samples = trajectory.samples;
    m_moments = trajectory.moments;
    m_tcp_path = trajectory.tcp_path;
    m_dt_ms = trajectory.dt_ms;
    m_trajectory_error = trajectory.error;
    m_sample = 0;
    emit trajectoryChanged();
    emit progressChanged();
    if (!m_trajectory_error.isEmpty())
    {
        SetStatus(tr("MoveJ failed after %1 samples").arg(momentCount()));
        emit errorOccurred(QStringLiteral("MoveJ: ") + m_trajectory_error);
        return;
    }
    SetStatus(tr("MoveJ: %1 samples, %2 s").arg(sampleCount()).arg(duration(), 0, 'f', 2));
    if (sampleCount() == 0)
        return;
    ShowSample(0);
    play();
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
    const int index = std::min(last, m_play_from + int(m_play_clock.elapsed() / m_dt_ms));
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
