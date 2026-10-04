#include "robotbackend.h"

#include "meshlayout.h"
#include "visuallayout.h"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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
    connect(this, &RobotBackend::requestPlan, m_worker, &SolverWorker::Plan);
    connect(this, &RobotBackend::requestApplySettings, m_worker, &SolverWorker::ApplySettings);
    connect(this, &RobotBackend::requestReload, m_worker, &SolverWorker::Initialize);
    connect(m_worker, &SolverWorker::SettingsReady, this, &RobotBackend::OnSettings);
    connect(m_worker, &SolverWorker::Initialized, this, &RobotBackend::OnInitialized);
    connect(m_worker, &SolverWorker::PoseReady, this, &RobotBackend::OnPose);
    connect(m_worker, &SolverWorker::TrajectoryReady, this, &RobotBackend::OnTrajectory);
    connect(m_worker, &SolverWorker::Failed, this, &RobotBackend::OnFailed);

    m_play_timer.setInterval(kPlayIntervalMs);
    connect(&m_play_timer, &QTimer::timeout, this, &RobotBackend::OnPlayTick);

    m_thread.setObjectName(QStringLiteral("solver"));
    m_thread.start();
    SetBusy(true);
    SetStatus(QT_TR_NOOP("Loading model from %1"), {m_data_dir});
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
    return {QVariantMap{{QStringLiteral("name"), QStringLiteral("obstacle")},
                        {QStringLiteral("source"), QUrl::fromLocalFile(QDir(m_data_dir).filePath(MeshLayout::ObstacleFile()))},
                        {QStringLiteral("obstacle"), true}}};
}

QVariantList RobotBackend::meshPoses() const
{
    return {ToNodePose(MeshLayout::ObstaclePlacement())};
}

int RobotBackend::visualPartCount() const
{
    return VisualLayout::Build(m_pose.frames, m_shape).size();
}

QVariantList RobotBackend::visualParts() const
{
    QVariantList out;
    for (const VisualLayout::Part &p : VisualLayout::Build(m_pose.frames, m_shape))
    {
        const bool sphere = p.shape == VisualLayout::Shape::Sphere;
        // Quick3D's #Cylinder / #Sphere are 100 units across (and tall), Y up
        const float diameter = 2 * p.radius / 100.f;
        const float length = sphere ? diameter : p.Length() / 100.f;
        out.append(QVariantMap{{QStringLiteral("shape"), sphere ? QStringLiteral("sphere") : QStringLiteral("cylinder")},
                               {QStringLiteral("role"), VisualLayout::RoleName(p.role)},
                               {QStringLiteral("joint"), p.joint},
                               {QStringLiteral("position"), QVariant::fromValue(p.Center())},
                               {QStringLiteral("rotation"), QVariant::fromValue(p.Rotation())},
                               {QStringLiteral("scale"), QVariant::fromValue(QVector3D(diameter, length, diameter))}});
    }
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

namespace
{
// `n` numbers of a list into out, false if the list does not hold exactly that
bool Numbers(const QVariant &value, double *out, int n)
{
    const QVariantList l = value.toList();
    if (l.size() != n)
        return false;
    for (int i = 0; i < n; i++)
    {
        bool ok = false;
        out[i] = l[i].toDouble(&ok);
        if (!ok || !std::isfinite(out[i]))
            return false;
    }
    return true;
}

QVariantList NumberList(const double *v, int n)
{
    QVariantList out;
    for (int i = 0; i < n; i++)
        out.append(v[i]);
    return out;
}

// a program step in UI units -> planner step; normalized: the same step with
// only the known keys and numbers as doubles
bool ParseStep(const QVariantMap &in, MotionStep &step, QVariantMap &normalized, QString &error)
{
    const QString type = in.value(QStringLiteral("type")).toString();
    bool ok_speed = false, ok_accel = false;
    const double speed = in.value(QStringLiteral("speed")).toDouble(&ok_speed);
    const double accel = in.value(QStringLiteral("accel")).toDouble(&ok_accel);
    if (!ok_speed || !ok_accel || !(speed > 0) || !(accel > 0))
    {
        error = QStringLiteral("speed and acceleration must be positive numbers");
        return false;
    }
    normalized = {{QStringLiteral("type"), type}, {QStringLiteral("speed"), speed}, {QStringLiteral("accel"), accel}};
    step = MotionStep();
    if (type == QLatin1String("J"))
    {
        double deg[6];
        if (!Numbers(in.value(QStringLiteral("joints")), deg, 6))
        {
            error = QStringLiteral("a J step needs 6 joint angles");
            return false;
        }
        step.kind = MotionStep::Joint;
        for (int i = 0; i < 6; i++)
            step.joints[i] = qDegreesToRadians(deg[i]);
        step.speed = qDegreesToRadians(speed);
        step.accel = qDegreesToRadians(accel);
        normalized[QStringLiteral("joints")] = NumberList(deg, 6);
        return true;
    }
    if (type != QLatin1String("L") && type != QLatin1String("C"))
    {
        error = QStringLiteral("unknown step type '%1' (J, L or C)").arg(type);
        return false;
    }
    double pose[6];
    if (!Numbers(in.value(QStringLiteral("pose")), pose, 6))
    {
        error = QStringLiteral("an %1 step needs a pose x, y, z, roll, pitch, yaw").arg(type);
        return false;
    }
    step.kind = type == QLatin1String("L") ? MotionStep::Linear : MotionStep::Circular;
    for (int i = 0; i < 6; i++)
        step.pose[i] = i < 3 ? pose[i] : qDegreesToRadians(pose[i]);
    step.speed = speed;
    step.accel = accel;
    normalized[QStringLiteral("pose")] = NumberList(pose, 6);
    if (step.kind == MotionStep::Circular)
    {
        double mid[3];
        if (!Numbers(in.value(QStringLiteral("mid")), mid, 3))
        {
            error = QStringLiteral("a C step needs a middle point x, y, z");
            return false;
        }
        for (int i = 0; i < 3; i++)
            step.mid[i] = mid[i];
        normalized[QStringLiteral("mid")] = NumberList(mid, 3);
    }
    return true;
}
}

void RobotBackend::Plan(const QString &name, const QVector<MotionStep> &steps)
{
    if (!m_ready || m_busy)
    {
        emit errorOccurred(tr("%1: the solver is busy").arg(name));
        return;
    }
    stop();
    m_plan_name = name;
    SetBusy(true);
    SetStatus(QT_TR_NOOP("Planning %1..."), {name});
    emit requestPlan(m_target_rad, steps, kTrajectoryDtMs);
}

void RobotBackend::moveJ(const QVariantList &target_deg, double speed_deg_s, double accel_deg_s2)
{
    MotionStep step;
    QVariantMap normalized;
    QString error;
    if (!ParseStep({{QStringLiteral("type"), QStringLiteral("J")}, {QStringLiteral("joints"), target_deg},
                    {QStringLiteral("speed"), speed_deg_s}, {QStringLiteral("accel"), accel_deg_s2}},
                   step, normalized, error))
    {
        emit errorOccurred(QStringLiteral("MoveJ: ") + error);
        return;
    }
    Plan(QStringLiteral("MoveJ"), {step});
}

void RobotBackend::moveL(const QVariantList &pose, double speed_m_s, double accel_m_s2)
{
    MotionStep step;
    QVariantMap normalized;
    QString error;
    if (!ParseStep({{QStringLiteral("type"), QStringLiteral("L")}, {QStringLiteral("pose"), pose},
                    {QStringLiteral("speed"), speed_m_s}, {QStringLiteral("accel"), accel_m_s2}},
                   step, normalized, error))
    {
        emit errorOccurred(QStringLiteral("MoveL: ") + error);
        return;
    }
    Plan(QStringLiteral("MoveL"), {step});
}

void RobotBackend::moveC(const QVariantList &mid, const QVariantList &pose, double speed_m_s, double accel_m_s2)
{
    MotionStep step;
    QVariantMap normalized;
    QString error;
    if (!ParseStep({{QStringLiteral("type"), QStringLiteral("C")}, {QStringLiteral("mid"), mid}, {QStringLiteral("pose"), pose},
                    {QStringLiteral("speed"), speed_m_s}, {QStringLiteral("accel"), accel_m_s2}},
                   step, normalized, error))
    {
        emit errorOccurred(QStringLiteral("MoveC: ") + error);
        return;
    }
    Plan(QStringLiteral("MoveC"), {step});
}

void RobotBackend::jogTcp(int axis, double step)
{
    if (axis < 0 || axis >= 6 || m_pose.tcp.size() != 6)
        return;
    QVariantList pose;
    for (double v : m_pose.tcp)
        pose.append(v);
    pose[axis] = pose[axis].toDouble() + step;
    // translations at 5 cm/s, pure turns at 0.5 rad/s (MoveL then times the angle)
    if (axis < 3)
        moveL(pose, 0.05, 0.25);
    else
        moveL(pose, 0.5, 1.0);
}

bool RobotBackend::addStep(const QVariantMap &step)
{
    MotionStep parsed;
    QVariantMap normalized;
    QString error;
    if (!ParseStep(step, parsed, normalized, error))
    {
        emit errorOccurred(tr("Program: %1").arg(error));
        return false;
    }
    m_program.append(normalized);
    emit programChanged();
    return true;
}

void RobotBackend::removeStep(int index)
{
    if (index < 0 || index >= m_program.size())
        return;
    m_program.removeAt(index);
    emit programChanged();
}

void RobotBackend::moveStep(int index, int delta)
{
    const int to = index + delta;
    if (index < 0 || index >= m_program.size() || to < 0 || to >= m_program.size() || delta == 0)
        return;
    m_program.move(index, to);
    emit programChanged();
}

void RobotBackend::clearProgram()
{
    if (m_program.isEmpty())
        return;
    m_program.clear();
    emit programChanged();
}

void RobotBackend::runProgram()
{
    if (m_program.isEmpty())
    {
        emit errorOccurred(tr("Program: no steps"));
        return;
    }
    QVector<MotionStep> steps;
    for (const QVariant &v : m_program)
    {
        MotionStep step;
        QVariantMap normalized;
        QString error;
        if (!ParseStep(v.toMap(), step, normalized, error))
        {
            emit errorOccurred(tr("Program: %1").arg(error));
            return;
        }
        steps.append(step);
    }
    Plan(tr("Program"), steps);
}

bool RobotBackend::saveProgram(const QUrl &url) const
{
    QFile file(url.isLocalFile() ? url.toLocalFile() : url.toString());
    if (!file.open(QIODevice::WriteOnly))
        return false;
    QJsonObject root{{QStringLiteral("format"), QStringLiteral("ManipStudio program")}, {QStringLiteral("version"), 1},
                     {QStringLiteral("steps"), QJsonArray::fromVariantList(m_program)}};
    return file.write(QJsonDocument(root).toJson()) > 0;
}

bool RobotBackend::loadProgram(const QUrl &url)
{
    QFile file(url.isLocalFile() ? url.toLocalFile() : url.toString());
    if (!file.open(QIODevice::ReadOnly))
    {
        emit errorOccurred(tr("Program: cannot open %1").arg(file.fileName()));
        return false;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    QVariantList loaded;
    for (const QVariant &v : doc.object().value(QStringLiteral("steps")).toArray().toVariantList())
    {
        MotionStep step;
        QVariantMap normalized;
        QString error;
        if (!ParseStep(v.toMap(), step, normalized, error))
        {
            emit errorOccurred(tr("Program: %1").arg(error));
            return false;
        }
        loaded.append(normalized);
    }
    m_program = loaded;
    emit programChanged();
    return true;
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

void RobotBackend::OnInitialized(const QVector<double> &lo, const QVector<double> &hi, double floor_z, const QVector<double> &max_moments,
                                 const std::vector<ArmPart> &shape)
{
    m_shape = shape;
    m_max_moments = max_moments;
    const bool reinit = m_ready;
    m_lo = lo;
    m_hi = hi;
    m_floor_z = floor_z;
    m_ready = true;
    SetBusy(false);
    // keep the arm inside the (possibly new) limits
    for (int i = 0; i < kJoints && i < m_lo.size(); i++)
        m_target_rad[i] = std::clamp(m_target_rad[i], m_lo[i], m_hi[i]);
    if (!reinit)
        SetStatus(QT_TR_NOOP("Model loaded, floor at z = %1 m"), {QString::number(floor_z)});
    emit readyChanged();
    emit requestPose(m_target_rad);
}

namespace
{
QVariantList ToList(const double *v, int n)
{
    QVariantList out;
    for (int i = 0; i < n; i++)
        out.append(v[i]);
    return out;
}

// reads `n` numbers from a QVariantList, false if it is not exactly that
bool FromList(const QVariant &value, double *out, int n)
{
    const QVariantList l = value.toList();
    if (l.size() != n)
        return false;
    for (int i = 0; i < n; i++)
    {
        bool ok = false;
        out[i] = l[i].toDouble(&ok);
        if (!ok)
            return false;
    }
    return true;
}

bool Number(const QVariant &value, double &out)
{
    bool ok = false;
    out = value.toDouble(&ok);
    return ok;
}
}

QVariantMap RobotBackend::settings() const
{
    if (!m_has_settings)
        return {};
    const RobotSettings &s = m_settings;
    QVariantList dh, limits, links;
    for (int i = 0; i < kJoints; i++)
    {
        dh.append(QVariantMap{{QStringLiteral("a"), s.dh[i].a},
                              {QStringLiteral("alpha"), qRadiansToDegrees(s.dh[i].alpha)},
                              {QStringLiteral("d"), s.dh[i].d}});
        limits.append(QVariantMap{{QStringLiteral("min"), qRadiansToDegrees(s.limit_min[i])},
                                  {QStringLiteral("max"), qRadiansToDegrees(s.limit_max[i])},
                                  {QStringLiteral("speed"), qRadiansToDegrees(s.max_speeds[i])},
                                  {QStringLiteral("accel"), qRadiansToDegrees(s.max_accels[i])}});
        links.append(QVariantMap{{QStringLiteral("mass"), s.links[i].mass},
                                 {QStringLiteral("com"), ToList(s.links[i].com.data(), 3)},
                                 {QStringLiteral("inertia"), ToList(s.links[i].inertia.data(), 6)}});
    }
    return {{QStringLiteral("dh"), dh},
            {QStringLiteral("limits"), limits},
            {QStringLiteral("maxMoments"), ToList(s.max_moments.data(), kJoints)},
            {QStringLiteral("floorZ"), s.floor_z},
            {QStringLiteral("links"), links},
            {QStringLiteral("payload"), QVariantMap{{QStringLiteral("mass"), s.payload.mass},
                                                    {QStringLiteral("com"), ToList(s.payload.com.data(), 3)}}},
            {QStringLiteral("jointRadii"), ToList(s.joint_radii.data(), kJoints)}};
}

void RobotBackend::applySettings(const QVariantMap &map, bool save)
{
    if (!m_ready || m_busy || !m_has_settings)
    {
        emit errorOccurred(tr("Settings: the solver is busy"));
        return;
    }
    RobotSettings s = m_settings;
    const QVariantList dh = map.value(QStringLiteral("dh")).toList();
    const QVariantList limits = map.value(QStringLiteral("limits")).toList();
    const QVariantList links = map.value(QStringLiteral("links")).toList();
    const QVariantMap payload = map.value(QStringLiteral("payload")).toMap();
    bool ok = dh.size() == kJoints && limits.size() == kJoints && links.size() == kJoints;
    for (int i = 0; ok && i < kJoints; i++)
    {
        const QVariantMap row = dh[i].toMap(), lim = limits[i].toMap(), link = links[i].toMap();
        double alpha = 0, lo = 0, hi = 0, speed = 0, accel = 0;
        ok = Number(row.value(QStringLiteral("a")), s.dh[i].a) && Number(row.value(QStringLiteral("alpha")), alpha) &&
             Number(row.value(QStringLiteral("d")), s.dh[i].d) && Number(lim.value(QStringLiteral("min")), lo) &&
             Number(lim.value(QStringLiteral("max")), hi) && Number(lim.value(QStringLiteral("speed")), speed) &&
             Number(lim.value(QStringLiteral("accel")), accel) && Number(link.value(QStringLiteral("mass")), s.links[i].mass) &&
             FromList(link.value(QStringLiteral("com")), s.links[i].com.data(), 3) &&
             FromList(link.value(QStringLiteral("inertia")), s.links[i].inertia.data(), 6);
        s.dh[i].alpha = qDegreesToRadians(alpha);
        s.limit_min[i] = qDegreesToRadians(lo);
        s.limit_max[i] = qDegreesToRadians(hi);
        s.max_speeds[i] = qDegreesToRadians(speed);
        s.max_accels[i] = qDegreesToRadians(accel);
    }
    ok = ok && FromList(map.value(QStringLiteral("maxMoments")), s.max_moments.data(), kJoints) &&
         Number(map.value(QStringLiteral("floorZ")), s.floor_z) && Number(payload.value(QStringLiteral("mass")), s.payload.mass) &&
         FromList(payload.value(QStringLiteral("com")), s.payload.com.data(), 3) &&
         FromList(map.value(QStringLiteral("jointRadii")), s.joint_radii.data(), kJoints);
    if (!ok)
    {
        emit errorOccurred(tr("Settings: every field needs a number"));
        return;
    }
    stop();
    SetBusy(true);
    SetStatus(save ? QT_TR_NOOP("Saving settings...") : QT_TR_NOOP("Applying settings..."));
    emit requestApplySettings(s, save);
}

void RobotBackend::reloadSettings()
{
    if (m_busy)
    {
        emit errorOccurred(tr("Settings: the solver is busy"));
        return;
    }
    stop();
    SetBusy(true);
    m_reloading = true;
    SetStatus(QT_TR_NOOP("Loading model from %1"), {m_data_dir});
    emit requestReload();
}

void RobotBackend::OnSettings(const RobotSettings &settings, bool saved)
{
    const bool reloaded = m_has_settings;
    m_settings = settings;
    m_has_settings = true;
    emit settingsChanged();
    if (reloaded)
    {
        // the old trajectory was planned with the old model
        ClearTrajectory();
        const QString file = QDir(m_data_dir).filePath(QStringLiteral("ManipulatorSettigns.ini"));
        if (m_reloading)
            SetStatus(QT_TR_NOOP("Settings reloaded from %1"), {file});
        else if (saved)
            SetStatus(QT_TR_NOOP("Settings saved to %1"), {file});
        else
            SetStatus(QT_TR_NOOP("Settings applied (not saved)"));
    }
    m_reloading = false;
}

void RobotBackend::ClearTrajectory()
{
    stop();
    m_samples.clear();
    m_moments.clear();
    m_tcp_path.clear();
    m_trajectory_error.clear();
    m_speed_limit.clear();
    m_sample = 0;
    emit trajectoryChanged();
    emit progressChanged();
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
    m_speed_limit.clear();
    if (trajectory.error.isEmpty() && trajectory.limit_joint >= 0)
        m_speed_limit = {{QStringLiteral("scale"), trajectory.speed_scale},
                         {QStringLiteral("joint"), trajectory.limit_joint},
                         {QStringLiteral("acceleration"), trajectory.limit_by_accel},
                         {QStringLiteral("step"), trajectory.limit_step}};
    m_sample = 0;
    emit trajectoryChanged();
    emit progressChanged();
    if (!m_trajectory_error.isEmpty())
    {
        SetStatus(QT_TR_NOOP("%1 failed after %2 samples"), {m_plan_name, QString::number(momentCount())});
        emit errorOccurred(m_plan_name + QStringLiteral(": ") + m_trajectory_error);
        return;
    }
    const QStringList done = {m_plan_name, QString::number(sampleCount()), QString::number(duration(), 'f', 2)};
    if (m_speed_limit.isEmpty())
        SetStatus(QT_TR_NOOP("%1: %2 samples, %3 s"), done);
    else
        SetStatus(trajectory.limit_by_accel ? QT_TR_NOOP("%1: %2 samples, %3 s, slowed to %4% by the q%5 acceleration limit")
                                            : QT_TR_NOOP("%1: %2 samples, %3 s, slowed to %4% by the q%5 speed limit"),
                  done + QStringList{QString::number(qRound(trajectory.speed_scale * 100)), QString::number(trajectory.limit_joint + 1)});
    if (sampleCount() == 0)
        return;
    ShowSample(0);
    play();
}

void RobotBackend::OnFailed(const QString &what, const QString &message)
{
    m_reloading = false;
    SetBusy(false);
    SetStatus(QT_TR_NOOP("%1 failed"), {what});
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

QString RobotBackend::status() const
{
    if (!m_status_source)
        return QString();
    QString text = tr(m_status_source);
    for (const QString &arg : m_status_args)
        text = text.arg(arg);
    return text;
}

void RobotBackend::SetStatus(const char *source, const QStringList &args)
{
    m_status_source = source;
    m_status_args = args;
    emit statusChanged();
}
