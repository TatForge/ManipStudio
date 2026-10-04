#include "motorhub.h"

#include "hardwaretransports.h"
#include "odriveprotocol.h"
#include "simulatedodrive.h"

#include <QSettings>
#include <QXYSeries>

#include <algorithm>
#include <cmath>

using namespace odrive;

namespace
{
constexpr double kHistorySeconds = 10;

bool Number(const QVariant &v, double &out)
{
    bool ok = false;
    const double d = v.toDouble(&ok);
    if (!ok || !std::isfinite(d))
        return false;
    out = d;
    return true;
}
}

MotorHub::MotorHub(const QString &config_file, QObject *parent) : QObject(parent), m_config_file(config_file)
{
    qRegisterMetaType<CanFrame>();
    m_clock.start();
    for (int i = 0; i < kMaxAxes; i++)
        m_axes[i].node = i;
    loadConfig();
    m_poll.setInterval(20);
    connect(&m_poll, &QTimer::timeout, this, &MotorHub::Poll);
    m_publish.setInterval(50);
    connect(&m_publish, &QTimer::timeout, this, [this] {
        if (m_dirty)
        {
            m_dirty = false;
            emit axesChanged();
        }
    });
    m_publish.start();
    SetStatus(QT_TR_NOOP("Not connected"));
}

MotorHub::~MotorHub()
{
    disconnectBus();
}

QStringList MotorHub::backends() const
{
    return QStringList{QStringLiteral("simulator")} + can_hardware::Backends();
}

QStringList MotorHub::interfaces(const QString &backend) const
{
    if (backend == QLatin1String("simulator"))
        return {QStringLiteral("6 axes")};
    return can_hardware::Interfaces(backend);
}

bool MotorHub::connectBus(const QString &backend, const QString &interface, int bitrate)
{
    // a failed attempt does not leave the previous bus connected
    disconnectBus();
    std::unique_ptr<CanTransport> transport;
    if (backend == QLatin1String("simulator"))
        transport = std::make_unique<SimulatedCanBus>(QVector<int>{0, 1, 2, 3, 4, 5}, false);
    else
    {
        QString error;
        transport = can_hardware::Create(backend, interface, bitrate, &error);
        if (!transport)
        {
            Fail(QT_TR_NOOP("Cannot connect: %1"), {error});
            return false;
        }
    }
    const QString name = backend == QLatin1String("simulator") ? backend
                                                               : QStringLiteral("%1 %2, %3 kbit/s").arg(backend, interface).arg(bitrate / 1000);
    return ConnectTransport(std::move(transport), name);
}

bool MotorHub::ConnectTransport(std::unique_ptr<CanTransport> transport, const QString &name)
{
    disconnectBus();
    QString error;
    if (!transport->Open(&error))
    {
        Fail(QT_TR_NOOP("Cannot connect: %1"), {error});
        return false;
    }
    m_bus = std::move(transport);
    m_bus_name = name;
    connect(m_bus.get(), &CanTransport::FrameReceived, this, &MotorHub::OnFrame);
    connect(m_bus.get(), &CanTransport::ErrorOccurred, this, [this](const QString &message) {
        Fail(QT_TR_NOOP("CAN error: %1"), {message});
    });
    for (Axis &a : m_axes)
    {
        a.online = false;
        a.last_seen = -1;
        a.has_position = false;
        a.jogging = false;
        a.target = qQNaN();
    }
    m_poll.start();
    SetStatus(QT_TR_NOOP("Connected to %1, waiting for the axes"), {name});
    emit connectedChanged();
    m_dirty = true;
    return true;
}

void MotorHub::disconnectBus()
{
    if (!m_bus)
        return;
    m_poll.stop();
    m_bus->Close();
    m_bus.reset();
    m_bus_name.clear();
    for (Axis &a : m_axes)
        a.online = false;
    SetStatus(QT_TR_NOOP("Not connected"));
    emit connectedChanged();
    m_dirty = true;
}

void MotorHub::setAxisCount(int count)
{
    count = std::clamp(count, 1, kMaxAxes);
    if (count == m_count)
        return;
    m_count = count;
    emit configChanged();
    m_dirty = true;
}

double MotorHub::ToDeg(const Axis &a, double turns) const
{
    return a.direction * (turns - a.zero) / a.gear * 360.0;
}

double MotorHub::ToTurns(const Axis &a, double deg) const
{
    return a.zero + a.direction * deg / 360.0 * a.gear;
}

QVariantList MotorHub::axes() const
{
    QVariantList out;
    for (int i = 0; i < m_count; i++)
    {
        const Axis &a = m_axes[i];
        const QPair<double, double> lim = i < m_limits.size() ? m_limits[i] : qMakePair(-360.0, 360.0);
        out.append(QVariantMap{
            {QStringLiteral("index"), i},
            {QStringLiteral("name"), Name(i)},
            {QStringLiteral("node"), a.node},
            {QStringLiteral("online"), a.online},
            {QStringLiteral("state"), a.state},
            {QStringLiteral("stateName"), a.online ? StateName(a.state) : QString()},
            {QStringLiteral("error"), a.error ? QStringLiteral("0x%1").arg(a.error, 0, 16) : QString()},
            {QStringLiteral("errorText"), ErrorText(a.error)},
            {QStringLiteral("closedLoop"), a.online && a.state == ClosedLoop},
            {QStringLiteral("busy"), a.online && a.state != Idle && a.state != ClosedLoop},
            {QStringLiteral("hasPosition"), a.has_position},
            {QStringLiteral("position"), a.has_position ? ToDeg(a, a.turns) : 0.0},
            {QStringLiteral("velocity"), a.direction * a.turns_s / a.gear * 360.0},
            {QStringLiteral("current"), a.iq},
            {QStringLiteral("busVoltage"), a.bus_voltage},
            {QStringLiteral("target"), a.target},
            {QStringLiteral("jogging"), a.jogging},
            {QStringLiteral("motorTurns"), a.turns},
            {QStringLiteral("gear"), a.gear},
            {QStringLiteral("direction"), a.direction},
            {QStringLiteral("zero"), a.zero},
            {QStringLiteral("maxSpeed"), a.max_speed},
            {QStringLiteral("maxAccel"), a.max_accel},
            {QStringLiteral("currentLimit"), a.current_limit},
            {QStringLiteral("min"), lim.first},
            {QStringLiteral("max"), lim.second},
        });
    }
    return out;
}

QString MotorHub::status() const
{
    if (!m_status_source)
        return {};
    QString s = tr(m_status_source);
    for (const QString &arg : m_status_args)
        s = s.arg(arg);
    return s;
}

void MotorHub::SetStatus(const char *source, const QStringList &args)
{
    m_status_source = source;
    m_status_args = args;
    emit statusChanged();
}

void MotorHub::Fail(const char *source, const QStringList &args)
{
    SetStatus(source, args);
    emit errorOccurred(status());
}

bool MotorHub::setAxisConfig(int axis, const QVariantMap &config)
{
    if (axis < 0 || axis >= kMaxAxes)
        return false;
    Axis next = m_axes[axis];
    double v = 0;
    bool ok = true;
    auto take = [&](const char *key, auto check, auto apply) {
        const QString k = QString::fromLatin1(key);
        if (!config.contains(k))
            return;
        if (!Number(config.value(k), v) || !check(v))
            ok = false;
        else
            apply(v);
    };
    take("node", [](double x) { return x >= 0 && x <= kMaxNode && x == std::floor(x); }, [&](double x) { next.node = int(x); });
    take("gear", [](double x) { return x != 0; }, [&](double x) { next.gear = x; });
    take("direction", [](double x) { return x == 1 || x == -1; }, [&](double x) { next.direction = int(x); });
    take("zero", [](double) { return true; }, [&](double x) { next.zero = x; });
    take("maxSpeed", [](double x) { return x > 0; }, [&](double x) { next.max_speed = x; });
    take("maxAccel", [](double x) { return x > 0; }, [&](double x) { next.max_accel = x; });
    take("currentLimit", [](double x) { return x > 0; }, [&](double x) { next.current_limit = x; });
    if (!ok)
    {
        Fail(QT_TR_NOOP("%1: invalid motor settings"), {Name(axis)});
        return false;
    }
    const bool node_changed = next.node != m_axes[axis].node;
    m_axes[axis] = next;
    Axis &a = m_axes[axis];
    if (node_changed)
    {
        a.online = false;
        a.last_seen = -1;
        a.has_position = false;
    }
    else if (a.online && a.state == ClosedLoop)
        SendLimits(a);
    emit configChanged();
    m_dirty = true;
    return true;
}

void MotorHub::setZeroHere(int axis)
{
    Axis *a = Get(axis, "zero");
    if (!a)
        return;
    if (!a->has_position)
    {
        Fail(QT_TR_NOOP("%1: no position from the motor yet"), {Name(axis)});
        return;
    }
    a->zero = a->turns;
    a->target = qQNaN();
    SetStatus(QT_TR_NOOP("%1: zero set at %2 motor turns"), {Name(axis), QString::number(a->zero, 'f', 4)});
    emit configChanged();
    m_dirty = true;
}

bool MotorHub::saveConfig()
{
    QSettings ini(m_config_file, QSettings::IniFormat);
    ini.beginGroup(QStringLiteral("motors"));
    ini.setValue(QStringLiteral("count"), m_count);
    for (int i = 0; i < kMaxAxes; i++)
    {
        const Axis &a = m_axes[i];
        const QString p = QString::number(i) + QLatin1Char('/');
        ini.setValue(p + QStringLiteral("node"), a.node);
        ini.setValue(p + QStringLiteral("gear"), a.gear);
        ini.setValue(p + QStringLiteral("direction"), a.direction);
        ini.setValue(p + QStringLiteral("zero_turns"), a.zero);
        ini.setValue(p + QStringLiteral("max_speed_deg_s"), a.max_speed);
        ini.setValue(p + QStringLiteral("max_accel_deg_s2"), a.max_accel);
        ini.setValue(p + QStringLiteral("current_limit_a"), a.current_limit);
    }
    ini.endGroup();
    ini.sync();
    if (ini.status() != QSettings::NoError)
    {
        Fail(QT_TR_NOOP("Cannot write %1"), {m_config_file});
        return false;
    }
    SetStatus(QT_TR_NOOP("Motor settings saved to %1"), {m_config_file});
    return true;
}

bool MotorHub::loadConfig()
{
    QSettings ini(m_config_file, QSettings::IniFormat);
    ini.beginGroup(QStringLiteral("motors"));
    m_count = std::clamp(ini.value(QStringLiteral("count"), m_count).toInt(), 1, kMaxAxes);
    for (int i = 0; i < kMaxAxes; i++)
    {
        const QString p = QString::number(i) + QLatin1Char('/');
        QVariantMap c;
        auto copy = [&](const char *key, const char *as) {
            if (ini.contains(p + QLatin1String(key)))
                c[QLatin1String(as)] = ini.value(p + QLatin1String(key));
        };
        copy("node", "node");
        copy("gear", "gear");
        copy("direction", "direction");
        copy("zero_turns", "zero");
        copy("max_speed_deg_s", "maxSpeed");
        copy("max_accel_deg_s2", "maxAccel");
        copy("current_limit_a", "currentLimit");
        if (!c.isEmpty())
            setAxisConfig(i, c);
    }
    ini.endGroup();
    emit configChanged();
    m_dirty = true;
    return true;
}

void MotorHub::setJointLimits(const QVariantList &limits)
{
    m_limits.clear();
    for (const QVariant &v : limits)
    {
        const QVariantMap m = v.toMap();
        m_limits.append({m.value(QStringLiteral("min")).toDouble(), m.value(QStringLiteral("max")).toDouble()});
    }
    m_dirty = true;
}

MotorHub::Axis *MotorHub::Get(int axis, const char *)
{
    if (axis < 0 || axis >= m_count)
        return nullptr;
    return &m_axes[axis];
}

bool MotorHub::Send(const CanFrame &frame)
{
    return m_bus && m_bus->Send(frame);
}

void MotorHub::SendLimits(const Axis &a)
{
    const double speed = a.max_speed * std::abs(TurnsPerDeg(a));
    const double accel = a.max_accel * std::abs(TurnsPerDeg(a));
    // the hard limit sits a little above the planned speed so that trajectories are not clipped
    Send(SetLimits(a.node, float(speed * 1.25), float(a.current_limit)));
    Send(SetTrajVelLimit(a.node, float(speed)));
    Send(SetTrajAccelLimits(a.node, float(accel), float(accel)));
}

void MotorHub::calibrate(int axis)
{
    Axis *a = Get(axis, "calibrate");
    if (!a || !a->online)
    {
        Fail(QT_TR_NOOP("%1: the motor is offline"), {Name(axis)});
        return;
    }
    if (a->state != Idle)
    {
        Fail(QT_TR_NOOP("%1: calibrate from IDLE only"), {Name(axis)});
        return;
    }
    Send(SetAxisState(a->node, FullCalibration));
    SetStatus(QT_TR_NOOP("%1: calibrating, the motor will beep and turn"), {Name(axis)});
}

void MotorHub::enable(int axis)
{
    Axis *a = Get(axis, "enable");
    if (!a || !a->online)
    {
        Fail(QT_TR_NOOP("%1: the motor is offline"), {Name(axis)});
        return;
    }
    if (!a->has_position)
    {
        Fail(QT_TR_NOOP("%1: no position from the motor yet"), {Name(axis)});
        return;
    }
    SendLimits(*a);
    Send(SetControllerMode(a->node, PositionControl, TrapTraj));
    Send(SetInputPos(a->node, float(a->turns)));
    Send(SetAxisState(a->node, ClosedLoop));
    a->target = ToDeg(*a, a->turns);
    a->jogging = false;
    SetStatus(QT_TR_NOOP("%1: closed loop requested"), {Name(axis)});
}

void MotorHub::disable(int axis)
{
    Axis *a = Get(axis, "disable");
    if (!a)
        return;
    Send(SetAxisState(a->node, Idle));
    a->jogging = false;
    a->target = qQNaN();
    SetStatus(QT_TR_NOOP("%1: idle, the motor is free"), {Name(axis)});
}

void MotorHub::clearErrors(int axis)
{
    Axis *a = Get(axis, "clear");
    if (!a)
        return;
    Send(ClearErrors(a->node));
    SetStatus(QT_TR_NOOP("%1: errors cleared"), {Name(axis)});
}

void MotorHub::moveTo(int axis, double deg)
{
    Axis *a = Get(axis, "move");
    if (!a || !a->online || a->state != ClosedLoop)
    {
        Fail(QT_TR_NOOP("%1: enable the motor first"), {Name(axis)});
        return;
    }
    if (!std::isfinite(deg))
        return;
    if (axis < m_limits.size())
    {
        const double clamped = std::clamp(deg, m_limits[axis].first, m_limits[axis].second);
        if (clamped != deg)
            SetStatus(QT_TR_NOOP("%1: target limited to %2°"), {Name(axis), QString::number(clamped)});
        deg = clamped;
    }
    a->jogging = false;
    Send(SetControllerMode(a->node, PositionControl, TrapTraj));
    Send(SetInputPos(a->node, float(ToTurns(*a, deg))));
    a->target = deg;
    m_dirty = true;
}

void MotorHub::jog(int axis, int direction)
{
    Axis *a = Get(axis, "jog");
    if (!a)
        return;
    if (direction == 0)
    {
        if (a->jogging)
            StopJog(*a);
        return;
    }
    if (!a->online || a->state != ClosedLoop)
    {
        Fail(QT_TR_NOOP("%1: enable the motor first"), {Name(axis)});
        return;
    }
    direction = direction > 0 ? 1 : -1;
    // never jog further out of the joint range
    if (axis < m_limits.size() && a->has_position)
    {
        const double deg = ToDeg(*a, a->turns);
        if ((direction > 0 && deg >= m_limits[axis].second) || (direction < 0 && deg <= m_limits[axis].first))
        {
            if (a->jogging)
                StopJog(*a);
            SetStatus(QT_TR_NOOP("%1: at the end of its range"), {Name(axis)});
            return;
        }
    }
    a->jog_deadline = m_clock.elapsed() + kJogTimeoutMs;
    if (a->jogging && a->jog_direction == direction)
        return;
    a->jogging = true;
    a->jog_direction = direction;
    a->target = qQNaN();
    const double turns_s = direction * a->direction * a->max_speed * TurnsPerDeg(*a);
    Send(SetControllerMode(a->node, VelocityControl, VelRamp));
    Send(SetInputVel(a->node, float(turns_s)));
    m_dirty = true;
}

void MotorHub::StopJog(Axis &a)
{
    a.jogging = false;
    Hold(a);
}

void MotorHub::Hold(Axis &a)
{
    if (!a.online || a.state != ClosedLoop)
        return;
    // stop with the configured deceleration where the motor can, then hold there
    const double decel = a.max_accel * std::abs(TurnsPerDeg(a));
    const double stop_at = a.turns + a.turns_s * std::abs(a.turns_s) / (2 * decel);
    Send(SetControllerMode(a.node, PositionControl, TrapTraj));
    Send(SetInputPos(a.node, float(stop_at)));
    a.target = ToDeg(a, stop_at);
    m_dirty = true;
}

void MotorHub::stopAll()
{
    for (int i = 0; i < m_count; i++)
    {
        m_axes[i].jogging = false;
        Hold(m_axes[i]);
    }
    SetStatus(QT_TR_NOOP("Stop: holding position"));
}

void MotorHub::disableAll()
{
    for (int i = 0; i < m_count; i++)
    {
        Send(SetAxisState(m_axes[i].node, Idle));
        m_axes[i].jogging = false;
        m_axes[i].target = qQNaN();
    }
    SetStatus(QT_TR_NOOP("All motors idle"));
}

void MotorHub::Record(QVector<QPointF> &history, double value)
{
    const double t = Now();
    history.append({t, value});
    while (!history.isEmpty() && history.first().x() < t - kHistorySeconds)
        history.removeFirst();
}

void MotorHub::OnFrame(const CanFrame &frame)
{
    const int node = NodeOf(frame.id);
    const Cmd cmd = Cmd(CmdOf(frame.id));
    for (int i = 0; i < m_count; i++)
    {
        Axis &a = m_axes[i];
        if (a.node != node || frame.rtr)
            continue;
        const bool was_online = a.online;
        a.last_seen = m_clock.elapsed();
        a.online = true;
        float first = 0, second = 0;
        switch (cmd)
        {
        case Cmd::Heartbeat:
        {
            Heartbeat h;
            if (!DecodeHeartbeat(frame, h))
                break;
            if (h.axis_error && h.axis_error != a.error)
                Fail(QT_TR_NOOP("%1: motor error %2"), {Name(i), ErrorText(h.axis_error)});
            if (a.state == ClosedLoop && h.state != ClosedLoop)
            {
                a.jogging = false;
                a.target = qQNaN();
            }
            a.error = h.axis_error;
            a.state = h.state;
            break;
        }
        case Cmd::GetEncoderEstimates:
            if (!DecodeFloats(frame, first, second))
                break;
            a.turns = first;
            a.turns_s = second;
            a.has_position = true;
            Record(a.position, ToDeg(a, a.turns));
            Record(a.velocity, a.direction * a.turns_s / a.gear * 360.0);
            if (std::isfinite(a.target))
                Record(a.target_history, a.target);
            break;
        case Cmd::GetIq:
            if (DecodeFloats(frame, first, second))
            {
                a.iq = second;
                Record(a.current, a.iq);
            }
            break;
        case Cmd::GetBusVoltageCurrent:
            if (DecodeFloats(frame, first, second))
                a.bus_voltage = first;
            break;
        default: break;
        }
        if (!was_online)
            SetStatus(QT_TR_NOOP("%1: motor online (node %2)"), {Name(i), QString::number(a.node)});
        m_dirty = true;
    }
}

void MotorHub::Poll()
{
    if (!m_bus)
        return;
    m_tick++;
    const qint64 now = m_clock.elapsed();
    for (int i = 0; i < m_count; i++)
    {
        Axis &a = m_axes[i];
        if (a.online && now - a.last_seen > kOfflineMs)
        {
            a.online = false;
            a.jogging = false;
            Fail(QT_TR_NOOP("%1: no answer from node %2"), {Name(i), QString::number(a.node)});
            m_dirty = true;
        }
        if (a.jogging)
        {
            const double deg = ToDeg(a, a.turns);
            const bool out_of_range = i < m_limits.size() &&
                                      ((a.jog_direction > 0 && deg >= m_limits[i].second) || (a.jog_direction < 0 && deg <= m_limits[i].first));
            // the button was released without a stop, or the range ends
            if (now > a.jog_deadline || out_of_range)
                StopJog(a);
        }
        Send(Request(a.node, Cmd::GetEncoderEstimates));
        if (m_tick % 5 == 0)
            Send(Request(a.node, Cmd::GetIq));
        if (m_tick % 25 == 0)
            Send(Request(a.node, Cmd::GetBusVoltageCurrent));
    }
}

QVector<QPointF> MotorHub::Series(int axis, const QString &kind) const
{
    if (axis < 0 || axis >= m_count)
        return {};
    const Axis &a = m_axes[axis];
    if (kind == QLatin1String("position"))
        return a.position;
    if (kind == QLatin1String("target"))
        return a.target_history;
    if (kind == QLatin1String("velocity"))
        return a.velocity;
    if (kind == QLatin1String("current"))
        return a.current;
    return {};
}

bool MotorHub::fillSeries(QObject *series, int axis, const QString &kind) const
{
    auto *xy = qobject_cast<QXYSeries *>(series);
    if (!xy)
        return false;
    xy->replace(Series(axis, kind));
    return true;
}

QVariantMap MotorHub::seriesRange(int axis, const QStringList &kinds) const
{
    const double t = Now();
    double lo = qInf(), hi = -qInf();
    for (const QString &kind : kinds)
        for (const QPointF &p : Series(axis, kind))
        {
            lo = std::min(lo, p.y());
            hi = std::max(hi, p.y());
        }
    if (!std::isfinite(lo))
    {
        lo = -1;
        hi = 1;
    }
    const double pad = std::max(0.5, (hi - lo) * 0.1);
    return {{QStringLiteral("xMin"), t - kHistorySeconds}, {QStringLiteral("xMax"), t},
            {QStringLiteral("yMin"), lo - pad}, {QStringLiteral("yMax"), hi + pad}};
}
